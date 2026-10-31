// 笛卡尔空间版：ARM_1 末端 X Y Z 力控、目标力 0，Rx Ry Rz 转动刚度 0 自由浮动，尽可能透明地手拖；ARM_2 全程不动。
// 关节空间版是 arm1_transparent_arm2_hold_rt.cpp，两个程序流程相同，方便对比手感。流程照官方
// example/intermediate6_realtime_cartesian_motion_force_control.cpp：
//   ① 回正：ARM_1 跑 Home 原语回到 home 姿态（--no-home 跳过）；
//   ② 受力回 0：ARM_1 跑 ZeroFTSensor 原语，把力/力矩传感器清零；力控就是靠测到的末端力，这一步必须做；
//   ③ 运动：切到实时模式 RT_CARTESIAN_MOTION_FORCE，ARM_1 可以手拖，一直到 Ctrl+C。
// ①② 期间 ARM_2 跑 Hold 原语。Home、ZeroFTSensor 只有原语版本，原语只能在非实时的 NRT_PRIMITIVE_EXECUTION 下运行，
// 控制模式又是整机共用的，所以 ①② 是非实时模式；从 ③ 开始到结束全程实时模式，不再切换。
// ① 之前机器人要在 IDLE：SetPassiveForceControl 文档写明只能在 IDLE 下调用。
//
// ③ 实时调度器每 1 ms 给两条臂各发一次末端指令（StreamCartesianMotionForce），由控制器内置的运动-力统一控制器跟踪：
//   ARM_1：kForceAxis 里为 1 的 X Y Z 轴做力控，目标力 0，手推多少就让多少；为 0 的轴做位置控制，锁在进入 ③ 时的位置。
//          Rx Ry Rz 只能做位置控制（力矩控制要 6 维力传感器），转动刚度设为 kRotStiffness，0 就是自由浮动。
//   ARM_2：6 轴全做位置控制，目标位姿一直是进入 ③ 时的末端位姿（官方 intermediate5 的 --hold），肘部由零空间控制拉住。
// 7 轴臂的肘部（零空间）不归末端力控管，由零空间姿态控制拉回进入 ③ 时的关节姿态，所以肘部不像关节版那样完全自由。
//
// 编译（项目根目录）：cmake -S script -B .build/script && cmake --build .build/script
// 运行（实时调度器需要 root）：sudo .build/script/arm1_transparent_arm2_hold_rt_cartesian <序列号> [选项]；--help 看选项。

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
constexpr size_t kCartDoF = rdk::kCartDoF;  // 末端自由度，6。

// ================================ 透明度参数：在这里调 ================================
// 都是 RDK 的官方参数，也可以用同名命令行参数临时覆盖，不用重新编译。3 维的按末端 X Y Z 或 Rx Ry Rz 排列。
// kPassiveForceControl：SetPassiveForceControl。false = 主动力控（RDK 默认）：闭环把末端测到的力压到 0，人推哪里臂就走哪里，
//   会多一点笛卡尔阻尼，有 kMaxLinearVel 限速保护。true = 被动力控：只前馈 0 力，不加阻尼，更透明，
//   但 kMaxLinearVel 不起作用，没人推时会自己漂，只剩 kArm1MaxJointVel 兜底。命令行 --passive。
constexpr bool kPassiveForceControl = false;
// kForceAxis：SetForceControlAxis 的 enabled_axes 的 X Y Z。1 = 这个轴力控，能拖；0 = 位置控制，锁在进入 ③ 时的位置。
//   RDK 默认全 0（纯位置控制）。Rx Ry Rz 不能开力控：10-30 实测报错 "moment control is only available with 6DoF F/T
//   sensor"，这台 Enlight-L 末端没有 6 维力传感器，末端力是关节力矩估算的。所以转动轴固定为位置控制，用 kRotStiffness 放开。
const std::array<bool, kCartDoF / 2> kForceAxis = {true, true, true};
// kRotStiffness：SetCartesianImpedance 的 K_x 后三个，Rx Ry Rz 的转动刚度 [Nm/rad]。0 = 该轴自由浮动，最透明
//   （文档："Setting motion stiffness of a motion-controlled Cartesian axis to 0 will make this axis free-floating"）；
//   范围 0 到 K_x_nom（启动时打印），等于 K_x_nom 就是锁住姿态。X Y Z 的刚度用 K_x_nom，只对锁住的轴起作用。
//   阻尼比 Z_x 用 RDK 默认 0.7：文档警告改它可能不稳定。
const std::array<double, kCartDoF / 2> kRotStiffness = {0.0, 0.0, 0.0};
// kMaxLinearVel：SetForceControlAxis 的 max_linear_vel，力控的 X Y Z 轴末端线速度上限 [m/s]，防止跑走。
//   范围 0.005–2.0，RDK 默认 1.0。越大越不拦手。只在主动力控下有效。
const std::array<double, kCartDoF / 2> kMaxLinearVel = {1.0, 1.0, 1.0};
// kNullSpaceRefTracking：SetNullSpaceObjectives 的 ref_positions_tracking，肘部（零空间）跟踪参考姿态的权重。
//   参考姿态是进入 ③ 时的关节位置。范围 0.1–1.0，RDK 默认 0.5；越小，肘部被拉回参考姿态越弱。
//   另外两个权重 linear/angular_manipulability 用 RDK 默认 0。
constexpr double kNullSpaceRefTracking = 0.5;
// kForceCtrlFrame：SetForceControlFrame 的力控坐标系，WORLD（RDK 默认）或 TCP。
//   X Y Z 全力控时没有区别；只在 kForceAxis 有 0 时决定锁哪个方向。
constexpr auto kForceCtrlFrame = rdk::CoordType::WORLD;
// 目标力 wrench_d 固定为 0，不是参数：力控轴的目标就是"手感到的力 = 0"。
// ====================================================================================

// ===== ① 回正、② 受力回 0：原语参数，名称、默认值、范围照抄 Flexiv Primitives 3E.1 文档，不影响手感 =====
constexpr int kHomeJntVelScale = 20;  // Home 的 jntVelScale：关节速度百分比，100 为全速。范围 1–100，文档默认 20。
// Home 的 target 不传，用文档默认的 home 姿态 0 -40 0 90 0 -50 0 deg。
// ZeroFTSensor（Basic Force Control 页）：清零时机器人要静止，不能被碰、不能接触环境。程序等 ARM_1 回正停稳后才清零。
constexpr double kZeroDataCollectTime = 0.2;  // dataCollectTime：采样多久来定零点 [s]。范围 0.1–1.0，文档默认 0.2。
constexpr int kZeroEnableStaticCheck = 0;  // enableStaticCheck：清零前检查机器人是否静止，1 检查。文档默认 0。
constexpr int kZeroCalibExtraPayload = 0;  // calibExtraPayload：标定时是否把额外负载算进工具负载，1 算。文档默认 0。

// ========== 安全检查和流程：不发给机器人、不影响手感，只决定什么时候整机停 ==========
constexpr double kArm1MaxJointVel = 1.0;  // 安全：ARM_1 任一关节超过它就整机停 [rad/s]。转动轴没有 kMaxLinearVel 那样的限速，靠它兜底。
// 安全：①②③ 全程 ARM_2 任一关节离开 hold 位置超过它就整机停 [rad]，0.01 rad ≈ 0.6°。结束时打印 ③ 期间 ARM_2 最大偏离和速度。
constexpr double kArm2MaxPosError = 0.01;
constexpr double kOperationalTimeout = 30.0;  // 流程：上伺服后等待就绪的最长时间 [s]。
constexpr double kHomeTimeout = 30.0;  // 流程：① 回正到位并停稳的最长时间 [s]。
constexpr double kZeroTimeout = 10.0;  // 流程：② 受力回 0 结束的最长时间 [s]。
constexpr auto kMonitorPeriod = std::chrono::milliseconds(10);  // 流程：主线程检查断线、E-stop、故障、控制模式的周期。
// 不做开始时的静止检查：没人推时慢慢漂（floating）不停机，只有超过 kArm1MaxJointVel 才停。

struct Params  // 命令行覆盖之后实际使用的参数。
{
    std::string robot_sn;
    bool passive = kPassiveForceControl;
    std::array<bool, kCartDoF / 2> force_axis = kForceAxis;
    std::array<double, kCartDoF / 2> rot_stiffness = kRotStiffness;
    std::array<double, kCartDoF / 2> max_linear_vel = kMaxLinearVel;
    double null_space_tracking = kNullSpaceRefTracking;
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
    std::cout << "用法：sudo .build/script/arm1_transparent_arm2_hold_rt_cartesian <序列号> [选项]\n"
                 "  <序列号>                    完整序列号，不要有空格，例如 Enlight-LL-123456\n"
                 "  --passive                   被动力控（覆盖 kPassiveForceControl）；不加是主动力控\n"
                 "  --force-axis AX AY AZ       覆盖 kForceAxis：X Y Z 各一个 0 或 1，1 = 能拖\n"
                 "  --rot-stiffness R1 R2 R3    覆盖 kRotStiffness：Rx Ry Rz 转动刚度 [Nm/rad]，0 = 自由\n"
                 "  --max-linear-vel V1 V2 V3   覆盖 kMaxLinearVel：X Y Z 末端线速度上限 [m/s]，范围 [0.005, 2]\n"
                 "  --null-space-tracking W     覆盖 kNullSpaceRefTracking：肘部拉回参考姿态的权重，范围 [0.1, 1]\n"
                 "  --no-home                   不先回正，在当前姿态受力回 0 后直接开始\n"
                 "  --clear-fault               启动时若有故障先尝试清除；不加则直接退出\n"
                 "  -h, --help                  显示本说明\n";
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
        if (arg == "--passive") {
            params.passive = true;
        } else if (arg == "--force-axis") {
            for (auto& flag : params.force_axis) {
                double value = 0.0;
                if (++i >= argc || !ParseNumber(argv[i], value) || (value != 0.0 && value != 1.0)) {
                    return "--force-axis 后面要跟 3 个 0 或 1";
                }
                flag = value == 1.0;
            }
        } else if (arg == "--rot-stiffness") {
            for (auto& value : params.rot_stiffness) {
                if (++i >= argc || !ParseNumber(argv[i], value)) {
                    return "--rot-stiffness 后面要跟 3 个数";
                }
            }
        } else if (arg == "--max-linear-vel") {
            for (auto& value : params.max_linear_vel) {
                if (++i >= argc || !ParseNumber(argv[i], value)) {
                    return "--max-linear-vel 后面要跟 3 个数";
                }
            }
        } else if (arg == "--null-space-tracking") {
            if (++i >= argc || !ParseNumber(argv[i], params.null_space_tracking)) {
                return "--null-space-tracking 后面要跟 1 个数";
            }
        } else if (arg == "--no-home") {
            params.home = false;
        } else if (arg == "--clear-fault") {
            params.clear_fault = true;
        } else {
            return "不认识的参数 " + arg;
        }
    }
    for (size_t i = 0; i < kCartDoF / 2; ++i) {  // 下发前按 RDK 文档范围检查；上方常量也一样要过这一关。
        if (!(params.max_linear_vel[i] >= 0.005 && params.max_linear_vel[i] <= 2.0)) {
            return "max_linear_vel 第 " + std::to_string(i + 1) + " 个值 " + Num(params.max_linear_vel[i])
                   + " 超出范围 [0.005, 2]";
        }
    }
    for (size_t i = 0; i < kCartDoF / 2; ++i) {  // 上限 K_x_nom 要连上机器人才知道，在 main 里检查。
        if (!(params.rot_stiffness[i] >= 0.0)) {
            return "rot_stiffness 第 " + std::to_string(i + 1) + " 个值 " + Num(params.rot_stiffness[i]) + " 不能小于 0";
        }
    }
    if (!(params.null_space_tracking >= 0.1 && params.null_space_tracking <= 1.0)) {
        return "null_space_tracking = " + Num(params.null_space_tracking) + " 超出范围 [0.1, 1]";
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

struct RtTargets  // ③ 的目标：进入 ③ 时两条臂的末端位姿，以及 ARM_2 的关节位置（只用来检查 ARM_2 动没动）。
{
    std::array<double, rdk::kPoseSize> arm1_pose = {};  // ARM_1 位置控制轴（kForceAxis 为 0）锁定的位姿。
    std::array<double, rdk::kPoseSize> arm2_pose = {};  // ARM_2 hold 的末端位姿。
    std::vector<double> arm2_q;  // ARM_2 hold 时的关节位置 [rad]。
};

// 实时任务：每 1 ms 检查一次，然后把两条臂的末端指令放进同一条 StreamCartesianMotionForce 发出去。
void PeriodicTask(rdk::Robot& robot, const RtTargets& targets, Arm2Record& record)
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
        const auto arm2_error = Error(arm2.q, targets.arm2_q);  // ARM_2 离 hold 位置的偏离 [rad]。
        for (size_t i = 0; i < kDoF; ++i) {
            record.max_error[i] = std::max(record.max_error[i], std::abs(arm2_error[i]));
            record.max_vel[i] = std::max(record.max_vel[i], std::abs(arm2.dtheta[i]));
        }
        CheckArm2(arm2_error);
        if (const auto over = OverLimit(arm1.dtheta, kArm1MaxJointVel, 2); !over.empty()) {  // 电机侧速度，噪声小。
            throw std::runtime_error("ARM_1 超过 kArm1MaxJointVel = " + Num(kArm1MaxJointVel) + " rad/s（" + over
                                     + "）：没人推时是自己在漂，被动力控就换回主动力控；手拖太快就调大 kArm1MaxJointVel");
        }

        // ARM_1：力控轴目标力 0，位置控制轴锁在 arm1_pose。ARM_2：只给位姿，就是纯位置控制（官方 intermediate5 的写法）。
        robot.StreamCartesianMotionForce({
            {rdk::JointGroup::ARM_1, rdk::RtCartesianCmd(targets.arm1_pose, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0})},
            {rdk::JointGroup::ARM_2, rdk::RtCartesianCmd(targets.arm2_pose)},
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
    if (info.all_groups.contains(rdk::JointGroup::EXT_AXIS)) {  // 本程序不处理外部轴。
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

std::string AxisStr(const std::array<bool, kCartDoF / 2>& axes)  // 例如 "X Y Z"，只列出为 1 的轴。
{
    static const std::array<std::string, kCartDoF / 2> kNames = {"X", "Y", "Z"};
    std::string text;
    for (size_t i = 0; i < kCartDoF / 2; ++i) {
        if (axes[i]) {
            text += (text.empty() ? "" : " ") + kNames[i];
        }
    }
    return text.empty() ? "无" : text;
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

    // 声明顺序决定析构顺序：调度器最先析构，实时任务不会用到已经销毁的 robot 和 targets。
    std::unique_ptr<rdk::Robot> robot;
    RtTargets targets;
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
        const auto info = robot->info();
        // ARM_1 的笛卡尔刚度：X Y Z 用名义刚度（只对锁住的轴起作用），Rx Ry Rz 用 kRotStiffness。动之前先查范围。
        const auto& arm1_K_nom = info.K_x_nom.at(rdk::JointGroup::ARM_1);
        auto arm1_K = arm1_K_nom;
        std::array<double, kCartDoF / 2> arm1_rot_K_nom;  // 只用来打印，告诉你锁住姿态用多大。
        for (size_t i = 0; i < kCartDoF / 2; ++i) {
            arm1_rot_K_nom[i] = arm1_K_nom[3 + i];
            if (params.rot_stiffness[i] > arm1_rot_K_nom[i]) {
                throw std::runtime_error("rot_stiffness 第 " + std::to_string(i + 1) + " 个值 " + Num(params.rot_stiffness[i])
                                         + " 超过 K_x_nom " + Num(arm1_rot_K_nom[i]) + " Nm/rad");
            }
            arm1_K[3 + i] = params.rot_stiffness[i];
        }

        need_stop = true;  // 从这里起结束时必须 Stop()。
        if (robot->mode() != rdk::Mode::IDLE) {
            robot->Stop();  // 机器人已静止，Stop() 只是切到 IDLE。
        }
        robot->SetPassiveForceControl(rdk::JointGroup::ARM_1, params.passive);  // 文档：只能在 IDLE 下调用。
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

        // ③ 从这里到结束全程实时模式。下面的设置照官方 intermediate5/6 的顺序：先切模式，再设置，再读起始位姿。
        robot->SwitchMode(rdk::Mode::RT_CARTESIAN_MOTION_FORCE);
        const auto& f = params.force_axis;
        robot->SetForceControlFrame(rdk::JointGroup::ARM_1, kForceCtrlFrame);
        robot->SetForceControlAxis(  // Rx Ry Rz 只能位置控制，见 kForceAxis 上方。
            rdk::JointGroup::ARM_1, {f[0], f[1], f[2], false, false, false}, params.max_linear_vel);
        robot->SetCartesianImpedance(rdk::JointGroup::ARM_1, arm1_K);  // 转动刚度 0 的轴自由浮动。
        robot->SetForceControlAxis(rdk::JointGroup::ARM_2, {false, false, false, false, false, false});  // 纯位置控制。
        robot->SetNullSpaceObjectives(rdk::JointGroup::ARM_1, 0.0, 0.0, params.null_space_tracking);
        const auto states = robot->states();
        targets.arm1_pose = states.at(rdk::JointGroup::ARM_1).tcp_pose;
        targets.arm2_pose = states.at(rdk::JointGroup::ARM_2).tcp_pose;
        targets.arm2_q = states.at(rdk::JointGroup::ARM_2).q;
        std::cout << "ARM_1：" << (params.passive ? "被动" : "主动") << "力控轴 " << AxisStr(params.force_axis) << "（"
                  << (kForceCtrlFrame == rdk::CoordType::WORLD ? "WORLD" : "TCP") << " 坐标系），线速度上限 ["
                  << rdk::utility::Arr2Str(params.max_linear_vel, 3) << "] m/s"
                  << (params.passive ? "（被动力控下不起作用）" : "") << "；Rx Ry Rz 转动刚度 ["
                  << rdk::utility::Arr2Str(params.rot_stiffness, 1) << "] Nm/rad（锁住用 K_x_nom ["
                  << rdk::utility::Arr2Str(arm1_rot_K_nom, 1) << "]）；肘部拉回权重 " << Num(params.null_space_tracking)
                  << std::endl;
        // 打印 ARM_2 离位置极限最近的关节，排查 hold 时被推动的原因。
        const auto& q_min = info.q_min.at(rdk::JointGroup::ARM_2);
        const auto& q_max = info.q_max.at(rdk::JointGroup::ARM_2);
        const auto margin = [&](size_t i) { return std::min(targets.arm2_q[i] - q_min[i], q_max[i] - targets.arm2_q[i]); };
        size_t nearest = 0;
        for (size_t i = 1; i < kDoF; ++i) {
            if (margin(i) < margin(nearest)) {
                nearest = i;
            }
        }
        std::cout << "ARM_2 hold 关节位置：[" << rdk::utility::Vec2Str(targets.arm2_q) << "] rad；离关节极限最近的是 J"
                  << nearest + 1 << "，还差 " << Num(margin(nearest)) << " rad（范围 [" << Num(q_min[nearest]) << ", "
                  << Num(q_max[nearest]) << "] rad）" << std::endl;
        scheduler->AddTask(
            [&] { PeriodicTask(*robot, targets, arm2_record); }, "HP periodic", 1, scheduler->max_priority());
        scheduler->Start();
        started = true;
        std::cout << "③ 实时模式 RT_CARTESIAN_MOTION_FORCE 已开始：ARM_1 可以手拖；ARM_2 hold 不动。按 Ctrl+C 结束。"
                  << std::endl;
        while (!g_stop) {
            if (const auto reason = NotRunningReason(*robot, rdk::Mode::RT_CARTESIAN_MOTION_FORCE); !reason.empty()) {
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
