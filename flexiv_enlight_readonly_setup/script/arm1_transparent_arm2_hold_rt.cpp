// ARM_1 尽可能透明地手拖，ARM_2 全程不动。流程照官方 AGENT_REFERENCE_primitives_only.py 的前两步再接实时浮动：
//   ① 回正：ARM_1 跑 Home 原语回到 home 姿态（--no-home 跳过）；
//   ② 受力回 0：ARM_1 跑 ZeroFTSensor 原语，把力/力矩传感器清零；
//   ③ 运动：切到实时模式 RT_JOINT_TORQUE，ARM_1 浮动可以手拖，一直到 Ctrl+C。
// ①② 期间 ARM_2 跑 Hold 原语。Home、ZeroFTSensor 只有原语版本，原语只能在非实时的 NRT_PRIMITIVE_EXECUTION 下运行，
// 控制模式又是整机共用的，所以 ①② 是非实时模式；从 ③ 开始到结束全程实时模式，不再切换。
//
// ③ 实时调度器每 1 ms 给两条臂各发一次关节力矩（StreamJointTorque），控制器在这个力矩上再加重力补偿和摩擦补偿：
//   ARM_1 浮动：力矩 = -阻尼 × 关节速度。算法照抄官方 example/intermediate4_realtime_joint_floating.cpp。
//   ARM_2 hold：力矩 = Kp × (起始位置 - 关节位置) - Kd × 关节速度，锁在进入实时模式那一刻的位置。
//               算法和增益照抄官方 example/intermediate3_realtime_joint_torque_control.cpp 的 --hold。
// 实时模式下没有 Hold 原语可用，ARM_2 的 hold 指令就是上面每 1 ms 发一次的力矩。
// ARM_1 没有导纳环（没有 responseTorque 死区、inertiaScale 这些原语参数），透明度只由下方"透明度参数"区决定。
//
// 编译（项目根目录）：cmake -S script -B .build/script && cmake --build .build/script
// 运行（实时调度器需要 root）：sudo .build/script/arm1_transparent_arm2_hold_rt <序列号> [选项]；--help 看选项。

#include <flexiv/rdk/robot.hpp>
#include <flexiv/rdk/scheduler.hpp>
#include <flexiv/rdk/utility.hpp>

#include <unistd.h>  // geteuid：检查是否以 root 运行。
// 输出用 std::cout 而不是 spdlog：/usr/local/include 里的 fmt 10 会盖住系统 spdlog 1.9 需要的 fmt 8，编不过。

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <functional>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

using namespace flexiv;

namespace {
constexpr size_t kDoF = rdk::kSerialJointDoF;  // 每条臂的关节数，7。

// ================================ 透明度参数：在这里调 ================================
// ARM_1 浮动只有这 2 个参数；也可以用同名命令行参数临时覆盖，不用重新编译。7 维的按关节 J1–J7 排列。
// kFloatingDamping：关节速度阻尼 [Nm·s/rad]，ARM_1 力矩 = -阻尼 × 关节速度。越小越轻，0 = 不加阻尼，最透明。
//   默认是官方 intermediate4 的值。范围 0 到 kHoldKd（官方 hold 增益，已知稳定），再大可能振荡。
//   阻尼也决定没人推时臂自己漂多快：静止时的模型误差力矩 e 让关节以 e / 阻尼 匀速漂（J2 误差 0.4 Nm、
//   阻尼 10 → 0.04 rad/s）；阻尼 0 时不是匀速而是越漂越快，直到超过 kArm1MaxJointVel 整机停。
const std::array<double, kDoF> kFloatingDamping = {10.0, 10.0, 5.0, 5.0, 1.0, 1.0, 1.0};
// kFrictionCompScale：RtJointTorqueCmd::friction_comp_scale，关节摩擦补偿百分比。范围 0–100，RDK 默认 100。
//   100 = 摩擦全部补偿，最透明；小于 100 相当于多加一份阻尼。
constexpr double kFrictionCompScale = 100.0;
// ARM_1 的 RtJointTorqueCmd 另外 2 个字段固定为 true，不是透明度参数：
//   enable_gravity_comp：重力补偿；关掉臂会掉下来。
//   enable_soft_limits：关节接近位置极限时软限位挡住，免得越限触发需要恢复操作的故障。
// ====================================================================================

// ===== ARM_2 hold：增益照抄官方 intermediate3 --hold（官方注明只作演示），不影响 ARM_1 手感 =====
// ARM_2 不开软限位：软限位会把靠近位置极限的关节往回推，和 hold 对着干。10-28 ARM_2 的 J7 在 6.15 rad 左右，
// 离极限 6.196 只差 0.04–0.05 rad；两次实测都是开始约 2 秒时只有 J7 被推离极限约 0.01 rad（最快 0.3 rad/s），
// 其他关节都不到 0.0005 rad。ARM_2 由 PD 锁在原位，离开 kArm2MaxPosError 就整机停，走不到极限，用不着软限位。
const std::array<double, kDoF> kHoldKp = {3000.0, 3000.0, 800.0, 800.0, 200.0, 200.0, 200.0};  // 位置增益 [Nm/rad]。
const std::array<double, kDoF> kHoldKd = {80.0, 80.0, 40.0, 40.0, 8.0, 8.0, 8.0};  // 速度增益 [Nm·s/rad]。

// ===== ① 回正、② 受力回 0：原语参数，名称、默认值、范围照抄 Flexiv Primitives 3E.1 文档，不影响手感 =====
constexpr int kHomeJntVelScale = 20;  // Home 的 jntVelScale：关节速度百分比，100 为全速。范围 1–100，文档默认 20。
// Home 的 target 不传，用文档默认的 home 姿态 0 -40 0 90 0 -50 0 deg。
// ZeroFTSensor（Basic Force Control 页）：清零时机器人要静止，不能被碰、不能接触环境。程序等 ARM_1 回正停稳后才清零。
constexpr double kZeroDataCollectTime = 0.2;  // dataCollectTime：采样多久来定零点 [s]。范围 0.1–1.0，文档默认 0.2。
constexpr int kZeroEnableStaticCheck = 0;  // enableStaticCheck：清零前检查机器人是否静止，1 检查。文档默认 0。
constexpr int kZeroCalibExtraPayload = 0;  // calibExtraPayload：标定时是否把额外负载算进工具负载，1 算。文档默认 0。

// ========== 安全检查和流程：不发给机器人、不影响手感，只决定什么时候整机停 ==========
constexpr double kArm1MaxJointVel = 1.0;  // 安全：ARM_1 任一关节超过它就整机停 [rad/s]。拦越漂越快；手拖得快可以调大。
// 安全：①②③ 全程 ARM_2 任一关节离开 hold 位置超过它就整机停 [rad]，0.01 rad ≈ 0.6°。看位置不看速度：③ 的 hold 是 PD，
// 不如 Hold 原语硬，10-28 第一次实测 2 秒时 J7 速度到 0.05 rad/s，原来的速度检查就停了。结束时打印 ③ 期间 ARM_2 最大偏离和速度。
constexpr double kArm2MaxPosError = 0.01;
constexpr double kOperationalTimeout = 30.0;  // 流程：上伺服后等待就绪的最长时间 [s]。
constexpr double kHomeTimeout = 30.0;  // 流程：① 回正到位并停稳的最长时间 [s]。
constexpr double kZeroTimeout = 10.0;  // 流程：② 受力回 0 结束的最长时间 [s]。
constexpr auto kMonitorPeriod = std::chrono::milliseconds(10);  // 流程：主线程检查断线、E-stop、故障、控制模式的周期。
// 不做开始时的静止检查：没人推时慢慢漂（floating）不停机，只有超过 kArm1MaxJointVel 才停。

struct Params  // 命令行覆盖之后实际使用的参数。
{
    std::string robot_sn;
    std::array<double, kDoF> damping = kFloatingDamping;
    double friction_comp_scale = kFrictionCompScale;
    bool home = true;
    bool clear_fault = false;
};

struct Arm2Record  // ③ 期间 ARM_2 最大偏离 hold 位置 [rad] 和最大速度 [rad/s]：实时任务写，调度器停下后主线程读。
{
    std::array<double, kDoF> max_error = {};
    std::array<double, kDoF> max_vel = {};
};

std::atomic<bool> g_stop = {false};  // 置 true 后实时任务不再发指令，主线程收尾。信号处理函数只做这一件事。
std::mutex g_reason_mutex;  // 保护 g_reason：实时线程和主线程都可能写。
std::string g_reason;  // 第一个停机原因；空 = Ctrl+C 正常结束。

void PrintHelp()
{
    std::cout << "用法：sudo .build/script/arm1_transparent_arm2_hold_rt <序列号> [选项]\n"
                 "  <序列号>                  完整序列号，不要有空格，例如 Enlight-LL-123456\n"
                 "  --damping D1 ... D7       覆盖 kFloatingDamping：ARM_1 关节速度阻尼 [Nm·s/rad]\n"
                 "  --friction-comp-scale S   覆盖 kFrictionCompScale：ARM_1 摩擦补偿百分比 [0, 100]\n"
                 "  --no-home                 不先回正，在当前姿态受力回 0 后直接浮动\n"
                 "  --clear-fault             启动时若有故障先尝试清除；不加则直接退出\n"
                 "  -h, --help                显示本说明\n";
}

std::string Num(double value)  // 数字转成最短的文字，例如 10、0.05。
{
    std::ostringstream oss;
    oss << value;
    return oss.str();
}

bool ParseNumber(const char* text, double& value)  // 整个字符串是一个有限的数才算成功。
{
    char* end = nullptr;
    value = std::strtod(text, &end);
    return end != text && *end == '\0' && std::isfinite(value);
}

std::string ParseArgs(int argc, char* argv[], Params& params)  // 读取命令行；未给出的用上方常量。出错返回原因。
{
    params.robot_sn = argv[1];
    if (params.robot_sn.starts_with("-")) {
        return "第一个参数必须是序列号";
    }
    for (int i = 2; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--damping") {
            for (auto& value : params.damping) {
                if (++i >= argc || !ParseNumber(argv[i], value)) {
                    return "--damping 后面要跟 7 个数";
                }
            }
        } else if (arg == "--friction-comp-scale") {
            if (++i >= argc || !ParseNumber(argv[i], params.friction_comp_scale)) {
                return "--friction-comp-scale 后面要跟 1 个数";
            }
        } else if (arg == "--no-home") {
            params.home = false;
        } else if (arg == "--clear-fault") {
            params.clear_fault = true;
        } else {
            return "不认识的参数 " + arg;
        }
    }
    for (size_t i = 0; i < kDoF; ++i) {  // 下发前检查范围；上方常量也一样要过这一关。
        if (!(params.damping[i] >= 0.0 && params.damping[i] <= kHoldKd[i])) {
            return "damping 第 " + std::to_string(i + 1) + " 个值 " + Num(params.damping[i]) + " 超出范围 [0, "
                   + Num(kHoldKd[i]) + "]";
        }
    }
    if (!(params.friction_comp_scale >= 0.0 && params.friction_comp_scale <= 100.0)) {
        return "friction_comp_scale = " + Num(params.friction_comp_scale) + " 超出范围 [0, 100]";
    }
    return "";
}

void RequestStop(const std::string& reason)  // 记下第一个停机原因并要求停机；实时线程和主线程都会调用。
{
    std::lock_guard<std::mutex> lock(g_reason_mutex);
    if (g_reason.empty()) {
        g_reason = reason;
    }
    g_stop = true;
}

void OnSignal(int)  // Ctrl+C、关闭终端、kill 都走同一个收尾流程。
{
    g_stop = true;
}

// 绝对值超过 limit 的关节，例如 "J2 1.05、J4 1.10"；都没超过返回空串。
template <typename Values>
std::string OverLimit(const Values& values, double limit, int decimals)
{
    bool over = false;
    for (size_t i = 0; i < kDoF; ++i) {
        over = over || std::abs(values[i]) > limit;
    }
    if (!over) {
        return "";  // 每 1 ms 都走这里，不分配内存。
    }
    std::ostringstream oss;
    oss.precision(decimals);
    oss << std::fixed;
    for (size_t i = 0; i < kDoF; ++i) {
        if (std::abs(values[i]) > limit) {
            oss << (oss.tellp() > 0 ? "、" : "") << "J" << i + 1 << " " << std::abs(values[i]);
        }
    }
    return oss.str();
}

std::array<double, kDoF> Error(const std::vector<double>& q, const std::vector<double>& target)  // q - target [rad]。
{
    std::array<double, kDoF> error;
    for (size_t i = 0; i < kDoF; ++i) {
        error[i] = q[i] - target[i];
    }
    return error;
}

void CheckArm2(const std::array<double, kDoF>& arm2_error)  // ARM_2 任一关节离开 hold 位置超过 kArm2MaxPosError 就抛出。
{
    if (const auto over = OverLimit(arm2_error, kArm2MaxPosError, 3); !over.empty()) {
        throw std::runtime_error("ARM_2 离开 hold 位置超过 kArm2MaxPosError = " + Num(kArm2MaxPosError) + " rad（"
                                 + over + " rad）：hold 没有保持住");
    }
}

// 实时任务：每 1 ms 读一次状态，算出两条臂的力矩，放进同一条 StreamJointTorque 发出去。
void PeriodicTask(
    rdk::Robot& robot, const Params& params, const std::vector<double>& arm2_hold_pos, Arm2Record& record)
{
    if (g_stop) {
        return;  // 要停了：不再发指令，等主线程停掉调度器后 Stop()。
    }
    try {
        if (robot.fault()) {
            throw std::runtime_error("机器人故障");
        }
        const auto all_states = robot.states();
        const auto& arm1 = all_states.at(rdk::JointGroup::ARM_1);
        const auto& arm2 = all_states.at(rdk::JointGroup::ARM_2);
        const auto arm2_error = Error(arm2.q, arm2_hold_pos);  // ARM_2 离 hold 位置的偏离 [rad]。
        for (size_t i = 0; i < kDoF; ++i) {
            record.max_error[i] = std::max(record.max_error[i], std::abs(arm2_error[i]));
            record.max_vel[i] = std::max(record.max_vel[i], std::abs(arm2.dtheta[i]));
        }
        CheckArm2(arm2_error);
        if (const auto over = OverLimit(arm1.dtheta, kArm1MaxJointVel, 2); !over.empty()) {  // 电机侧速度，噪声小。
            throw std::runtime_error("ARM_1 超过 kArm1MaxJointVel = " + Num(kArm1MaxJointVel) + " rad/s（" + over
                                     + "）：没人推时是自己越漂越快，调大 kFloatingDamping；手拖太快就调大 kArm1MaxJointVel");
        }

        std::vector<double> arm1_torque(kDoF), arm2_torque(kDoF);
        for (size_t i = 0; i < kDoF; ++i) {
            arm1_torque[i] = -params.damping[i] * arm1.dtheta[i];
            arm2_torque[i] = kHoldKp[i] * (arm2_hold_pos[i] - arm2.q[i]) - kHoldKd[i] * arm2.dtheta[i];
        }
        // ARM_1：重力补偿、软限位都开。ARM_2：只开重力补偿，不开软限位（原因见 kHoldKp 上方），摩擦补偿用 RDK 默认 100。
        robot.StreamJointTorque({
            {rdk::JointGroup::ARM_1, rdk::RtJointTorqueCmd(arm1_torque, true, true, params.friction_comp_scale)},
            {rdk::JointGroup::ARM_2, rdk::RtJointTorqueCmd(arm2_torque, true, false)},
        });
    } catch (const std::exception& e) {
        RequestStop(e.what());
    }
}

std::string NotRunningReason(const rdk::Robot& robot, rdk::Mode mode)  // 机器人不能继续在 mode 下运行的原因；正常返回空串。
{
    if (!robot.connected()) {
        return "与机器人断线";
    }
    if (!robot.estop_released()) {
        return "E-stop 被按下";
    }
    if (robot.fault()) {
        return "机器人故障";
    }
    if (!robot.operational()) {
        return "机器人未就绪：" + rdk::kOpStatusNames.at(robot.operational_status());
    }
    if (robot.mode() != mode) {
        return "控制模式被切到 " + rdk::kModeNames.at(robot.mode());
    }
    if (robot.reached_timeliness_failure_limit()) {
        return "实时指令迟到太多：关掉占 CPU 的程序再试";
    }
    return "";
}

void Prepare(rdk::Robot& robot, const Params& params)  // 连接后的检查和上伺服；不满足就抛出，此时还没发过运动指令。
{
    if (robot.fault()) {
        if (!params.clear_fault) {
            throw std::runtime_error("机器人有故障；确认原因后加 --clear-fault 重试");
        }
        if (!robot.ClearFault()) {
            throw std::runtime_error("故障无法清除");
        }
    }
    const auto info = robot.info();
    for (const auto group : {rdk::JointGroup::ARM_1, rdk::JointGroup::ARM_2}) {
        if (!info.single_arm_groups.contains(group)) {
            throw std::runtime_error("控制器没有单臂关节组 " + rdk::kJointGroupNames.at(group));
        }
        if (info.DoF.at(group) != kDoF) {
            throw std::runtime_error(rdk::kJointGroupNames.at(group) + " 不是 7 个关节");
        }
    }
    if (info.all_groups.contains(rdk::JointGroup::EXT_AXIS)) {  // 实时模式下外部轴也要每 1 ms 给指令，这里不处理。
        throw std::runtime_error("机器人有外部轴；本程序只给 ARM_1、ARM_2 发指令");
    }
    if (!robot.all_stopped()) {  // 有别的程序在动机器人时不接管。
        throw std::runtime_error("机器人还在运动；先停止其他控制程序");
    }
    if (!robot.operational()) {
        robot.ServoOn();  // 上伺服；E-stop 必须已松开。
        const auto deadline
            = std::chrono::steady_clock::now() + std::chrono::duration<double>(kOperationalTimeout);
        while (!robot.operational()) {
            if (g_stop) {
                throw std::runtime_error("等待机器人就绪时被中断");
            }
            if (std::chrono::steady_clock::now() > deadline) {
                throw std::runtime_error("等待机器人就绪超时（检查 E-stop、AUTO(Remote)）");
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    }
}

// ①② 用：ARM_1 跑 arm1_primitive、ARM_2 跑 Hold，放进同一条 ExecutePrimitive，然后每 kMonitorPeriod 检查一次直到 done 为真。
// done 收到 ARM_1 这条原语的状态，它已经不在运行时收到 nullptr。超时、机器人不能运行、ARM_2 离开 arm2_start、Ctrl+C 都抛出。
void RunArm1Primitive(rdk::Robot& robot, const rdk::PrimitiveArgs& arm1_primitive, const std::vector<double>& arm2_start,
    double timeout, const std::function<bool(const rdk::PrimitiveStates*)>& done)
{
    std::map<rdk::JointGroup, rdk::PrimitiveArgs> commands = {
        {rdk::JointGroup::ARM_1, arm1_primitive},
        {rdk::JointGroup::ARM_2, rdk::PrimitiveArgs("Hold", {})},  // Hold 没有输入参数。
    };
    for (auto& entry : commands) {
        entry.second.external_axis_control = false;  // 不带动外部轴。
        entry.second.sync_motion_mode = rdk::SyncMotionMode::DISABLE;  // 两臂互不跟随。
    }
    robot.ExecutePrimitive(commands);  // 阻塞到两条原语开始执行。
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout);
    while (true) {
        if (g_stop) {
            throw std::runtime_error(arm1_primitive.pt_name + " 期间被中断");
        }
        if (const auto reason = NotRunningReason(robot, rdk::Mode::NRT_PRIMITIVE_EXECUTION); !reason.empty()) {
            throw std::runtime_error(reason);
        }
        CheckArm2(Error(robot.states().at(rdk::JointGroup::ARM_2).q, arm2_start));
        const auto states = robot.primitive_states();  // 只包含正在运行原语的关节组。
        const auto it = states.find(rdk::JointGroup::ARM_1);
        const bool running = it != states.end() && it->second.pt_name == arm1_primitive.pt_name;
        if (done(running ? &it->second : nullptr)) {
            return;
        }
        if (std::chrono::steady_clock::now() > deadline) {
            throw std::runtime_error("ARM_1 " + arm1_primitive.pt_name + " " + Num(timeout) + " 秒内没有完成");
        }
        std::this_thread::sleep_for(kMonitorPeriod);
    }
}

void PrintArm1Force(const rdk::Robot& robot, const std::string& when)  // 打印 ARM_1 外力矩和末端外力，对比受力回 0 前后。
{
    const auto arm1 = robot.states().at(rdk::JointGroup::ARM_1);
    std::cout << "ARM_1 受力回 0 " << when << "：外力矩 tau_ext [" << rdk::utility::Vec2Str(arm1.tau_ext, 2)
              << "] Nm，末端外力 tcp_wrench [" << rdk::utility::Arr2Str(arm1.tcp_wrench, 2) << "] N/Nm" << std::endl;
}
}  // namespace

int main(int argc, char* argv[])
{
    if (argc < 2 || rdk::utility::ProgramArgsExistAny(argc, argv, {"-h", "--help"})) {
        PrintHelp();
        return argc < 2 ? 2 : 0;
    }
    Params params;
    if (const auto error = ParseArgs(argc, argv, params); !error.empty()) {
        std::cerr << "参数错误：" << error << "\n\n";
        PrintHelp();
        return 2;
    }
    if (geteuid() != 0) {  // RDK README：用到 rdk::Scheduler 的程序必须以 root 运行。
        std::cerr << "实时调度器需要 root 权限：sudo " << argv[0] << " ...\n";
        return 2;
    }

    // 声明顺序决定析构顺序：调度器最先析构，实时任务不会用到已经销毁的 robot 和 arm2_hold_pos。
    std::unique_ptr<rdk::Robot> robot;
    std::vector<double> arm2_hold_pos;  // ARM_2 锁定的关节位置 [rad]。
    Arm2Record arm2_record;
    std::unique_ptr<rdk::Scheduler> scheduler;
    bool need_stop = false, started = false;
    try {
        scheduler = std::make_unique<rdk::Scheduler>();  // 先建调度器：系统问题在连机器人之前就暴露。
        robot = std::make_unique<rdk::Robot>(params.robot_sn);
        for (int sig : {SIGINT, SIGTERM, SIGHUP}) {  // 在 RDK 初始化之后装，保证由这里接管信号。
            std::signal(sig, OnSignal);
        }
        Prepare(*robot, params);

        need_stop = true;  // 从这里起结束时必须 Stop()。
        robot->SwitchMode(rdk::Mode::NRT_PRIMITIVE_EXECUTION);  // ①② 用原语，只能在这个非实时模式下运行。
        const auto arm2_start = robot->states().at(rdk::JointGroup::ARM_2).q;  // ①② 期间 ARM_2 不能离开这里。
        if (params.home) {
            std::cout << "① ARM_1 回正中（Home，关节速度 " << kHomeJntVelScale << "%），ARM_2 Hold。Ctrl+C 可随时停 ……"
                      << std::endl;
            RunArm1Primitive(*robot, rdk::PrimitiveArgs("Home", {{"jntVelScale", kHomeJntVelScale}}), arm2_start,
                kHomeTimeout, [&](const rdk::PrimitiveStates* state) {
                    if (!state) {
                        throw std::runtime_error("ARM_1 Home 没到位就结束了");
                    }
                    return rdk::utility::PrimitiveStateTrue(*state, "reachedTarget")
                           && robot->stopped().at(rdk::JointGroup::ARM_1);  // 到位并停稳，才能接着清零。
                });
            std::cout << "ARM_1 已回正。" << std::endl;
        }
        std::cout << "② ARM_1 受力回 0 中（ZeroFTSensor），不要碰 ARM_1 ……" << std::endl;
        PrintArm1Force(*robot, "前");
        RunArm1Primitive(*robot,
            rdk::PrimitiveArgs("ZeroFTSensor", {{"dataCollectTime", kZeroDataCollectTime},
                                                   {"enableStaticCheck", kZeroEnableStaticCheck},
                                                   {"calibExtraPayload", kZeroCalibExtraPayload}}),
            arm2_start, kZeroTimeout, [](const rdk::PrimitiveStates* state) {
                return !state || rdk::utility::PrimitiveStateTrue(*state, "terminated");  // 文档默认转移条件。
            });
        PrintArm1Force(*robot, "后");

        robot->SwitchMode(rdk::Mode::RT_JOINT_TORQUE);  // ③ 从这里到结束全程实时模式。
        arm2_hold_pos = robot->states().at(rdk::JointGroup::ARM_2).q;
        std::cout << "ARM_1 浮动参数：kFloatingDamping = [" << rdk::utility::Arr2Str(params.damping, 2)
                  << "] Nm·s/rad，kFrictionCompScale = " << Num(params.friction_comp_scale) << std::endl;
        const auto info = robot->info();  // 打印 ARM_2 离位置极限最近的关节，排查 hold 时被推动的原因。
        const auto& q_min = info.q_min.at(rdk::JointGroup::ARM_2);
        const auto& q_max = info.q_max.at(rdk::JointGroup::ARM_2);
        const auto margin = [&](size_t i) { return std::min(arm2_hold_pos[i] - q_min[i], q_max[i] - arm2_hold_pos[i]); };
        size_t nearest = 0;
        for (size_t i = 1; i < kDoF; ++i) {
            if (margin(i) < margin(nearest)) {
                nearest = i;
            }
        }
        std::cout << "ARM_2 hold 位置：[" << rdk::utility::Vec2Str(arm2_hold_pos) << "] rad；离关节极限最近的是 J"
                  << nearest + 1 << "，还差 " << Num(margin(nearest)) << " rad（范围 [" << Num(q_min[nearest]) << ", "
                  << Num(q_max[nearest]) << "] rad）" << std::endl;
        scheduler->AddTask([&] { PeriodicTask(*robot, params, arm2_hold_pos, arm2_record); }, "HP periodic", 1,
            scheduler->max_priority());
        scheduler->Start();
        started = true;
        std::cout << "③ 实时模式 RT_JOINT_TORQUE 已开始：ARM_1 可以手拖；ARM_2 hold 不动。按 Ctrl+C 结束。" << std::endl;
        while (!g_stop) {
            if (const auto reason = NotRunningReason(*robot, rdk::Mode::RT_JOINT_TORQUE); !reason.empty()) {
                RequestStop(reason);
            }
            std::this_thread::sleep_for(kMonitorPeriod);
        }
    } catch (const std::exception& e) {
        RequestStop(e.what());
    }

    g_stop = true;  // 出错路径上也让实时任务停止发指令。
    for (int sig : {SIGINT, SIGTERM, SIGHUP}) {  // 停机过程中不再被信号打断。
        std::signal(sig, SIG_IGN);
    }
    bool record_ready = false;  // 调度器正常停下后，arm2_record 才不会再被实时任务改。
    if (started) {
        try {
            scheduler->Stop();  // 先停实时任务，再整机 Stop()。
            record_ready = true;
        } catch (const std::exception& e) {
            std::cout << "调度器没有正常停止：" << e.what() << std::endl;
        }
    }
    if (need_stop) {
        try {
            robot->Stop();  // 整机停到 IDLE，两臂都保持当前位置。
            std::cout << "已 Stop()：机器人进入 IDLE 并保持当前位置。" << std::endl;
        } catch (const std::exception& e) {  // E-stop 按下时 Stop() 无法执行，机器人已由 E-stop 停住。
            std::cout << "Stop() 没有执行：" << e.what() << std::endl;
        }
    }
    if (record_ready) {
        std::cout << "③ 期间 ARM_2 最大偏离 hold 位置：[" << rdk::utility::Arr2Str(arm2_record.max_error, 4)
                  << "] rad；最大速度：[" << rdk::utility::Arr2Str(arm2_record.max_vel, 3) << "] rad/s" << std::endl;
    }

    std::lock_guard<std::mutex> lock(g_reason_mutex);
    if (!g_reason.empty()) {
        std::cerr << "已停止：" << g_reason << std::endl;
        return 1;
    }
    return 0;
}
