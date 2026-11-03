// RT 六维笛卡尔浮动：ARM_1 末端 X Y Z、Rx Ry Rz 六个方向都能手拖，尽可能透明；ARM_2 全程不动。
// 控制方法照所附方案（原始版本 third_party/flexiv_rdk/example/intermediate8_realtime_cartesian_floating.cpp，不改它），
// 运行流程照 arm1_transparent_arm2_hold_rt.cpp：
//   ① 回正：ARM_1 跑 Home 原语回到 home 姿态（--no-home 跳过）；
//   ② 受力回 0：ARM_1 跑 ZeroFTSensor 原语，把力/力矩传感器清零；
//   ③ 运动：切到实时模式 RT_CARTESIAN_MOTION_FORCE，ARM_1 浮动可以手拖，一直到 Ctrl+C。
// 给 ARM_1 发指令的同时一直给 ARM_2 发 hold：①② 每条原语指令都是 {ARM_1 的原语, ARM_2 Hold}，③ 每次实时指令都同时带两条臂。
// Home、ZeroFTSensor 只有原语版本，原语只能在非实时的 NRT_PRIMITIVE_EXECUTION 下运行，控制模式又是整机共用的，
// 所以 ①② 是非实时模式；从 ③ 开始到结束全程实时模式，不再切换。
// ① 之前机器人要在 IDLE：SetPassiveForceControl 文档写明只能在 IDLE 下调用。
//
// ③ 实时调度器每 1 ms 给两条臂各发一次末端指令（StreamCartesianMotionForce），由控制器内置的运动-力统一控制器跟踪：
//   ARM_1：目标位姿一直是进入 ③ 时的末端位姿、目标力 0；不跟随实测位姿，也不自己解 IK，浮动靠的是刚度降到 0。
//          先用名义刚度 K_x_nom 成功发满 kWarmupCycles 次，确认实时指令一直在发，再把六轴刚度降到 K_x_nom × kStiffnessScale，
//          或者 --passive 开六轴被动零力控制。降刚度、开力控都是实时模式里的参数设置，不切换控制模式，两条臂的指令照常发。
//   ARM_2：6 轴纯位置控制、名义刚度，目标位姿一直是进入 ③ 时的末端位姿（官方 intermediate5 的 --hold）。
//          实时模式下没有 Hold 原语可用，ARM_2 的 hold 指令就是每 1 ms 发一次的这个位姿。
// 7 轴臂的肘部（零空间）不归末端刚度管，由零空间姿态控制拉回进入 ③ 时的关节姿态，所以肘部不像关节版那样完全自由。
//
// 编译（项目根目录）：cmake -S script -B .build/script && cmake --build .build/script
// 运行（实时调度器需要 root）：sudo .build/script/intermediate8_realtime_cartesian_floating <序列号> [选项]；--help 看选项。

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
#include <cstdint>
#include <cstdlib>
#include <fstream>
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
// 都是 RDK 的官方参数，也可以用命令行参数临时覆盖，不用重新编译。
// kStiffnessScale：ARM_1 六轴笛卡尔刚度 = K_x_nom × kStiffnessScale（SetCartesianImpedance 的 K_x，K_x_nom 启动时打印）。
//   范围 0–1；0 = 六维零刚度浮动，最透明（文档："Setting motion stiffness of a motion-controlled Cartesian axis to 0 will
//   make this axis free-floating"）。方案建议 0.10 → 0.05 → 0.02 → 0.00 一档一档降。命令行 --scale=S。
//   阻尼比 Z_x 用 RDK 默认 0.7：文档警告改它可能不稳定。
constexpr double kStiffnessScale = 0.10;
// kNullSpaceRefTracking：SetNullSpaceObjectives 的 ref_positions_tracking，肘部（零空间）拉回进入 ③ 时关节姿态的权重。
//   范围 0.1–1.0，RDK 默认 0.5；越小肘部越自由。0.1 是最小值，不等于关掉零空间控制。另外两个权重用 RDK 默认 0。
//   命令行 --nullspace=W。
constexpr double kNullSpaceRefTracking = 0.5;
// kPassiveForceControl：true = 六轴被动零力控制（方案的 passive zero-wrench）：SetPassiveForceControl 打开，六轴都做力控、
//   目标力 0，只前馈 0 力，不加主动力控的阻尼；此时 kStiffnessScale 不起作用，SetForceControlAxis 的线速度保护也不起作用。
//   Rx Ry Rz 力矩控制要 6 维力传感器：10-30 实测这台报错 "moment control is only available with 6DoF F/T sensor"，
//   所以启动时检查到没有力传感器就退出。命令行 --passive。
constexpr bool kPassiveForceControl = false;
// 目标位姿固定为进入 ③ 时的位姿、目标力固定为 0，不是参数。
// ====================================================================================

// ===== ① 回正、② 受力回 0：原语参数，同 arm1_transparent_arm2_hold_rt.cpp，照抄 Flexiv Primitives 3E.1 文档，不影响手感 =====
constexpr int kHomeJntVelScale = 20;  // Home 的 jntVelScale：关节速度百分比，100 为全速。范围 1–100，文档默认 20。
// Home 的 target 不传，用文档默认的 home 姿态 0 -40 0 90 0 -50 0 deg。
// ZeroFTSensor（Basic Force Control 页）：清零时机器人要静止，不能被碰、不能接触环境。程序等 ARM_1 回正停稳后才清零。
constexpr double kZeroDataCollectTime = 0.2;  // dataCollectTime：采样多久来定零点 [s]。范围 0.1–1.0，文档默认 0.2。
constexpr int kZeroEnableStaticCheck = 0;  // enableStaticCheck：清零前检查机器人是否静止，1 检查。文档默认 0。
constexpr int kZeroCalibExtraPayload = 0;  // calibExtraPayload：标定时是否把额外负载算进工具负载，1 算。文档默认 0。

// ========== 安全检查和流程：不发给机器人、不影响手感，只决定什么时候整机停 ==========
// ARM_1 的 4 项检查和阈值照所附方案，是调试用的示例值，不是 Flexiv 认证的安全限值。
constexpr double kMaxTcpLinearVel = 0.20;  // 安全：③ ARM_1 末端线速度超过它就整机停 [m/s]。
constexpr double kMaxTcpAngularVel = 0.70;  // 安全：③ ARM_1 末端角速度超过它就整机停 [rad/s]。
constexpr double kMaxTcpDisplacement = 0.20;  // 安全：③ ARM_1 末端离开进入 ③ 时的位置超过它就整机停 [m]。
// 安全：③ ARM_1 任一关节离位置极限不到它就整机停 [rad]；进入 ③ 时就不到，不开始。
constexpr double kJointLimitMargin = 0.12;
constexpr double kJointVelFraction = 0.30;  // 安全：③ ARM_1 任一关节速度超过 dq_max 的这个比例就整机停。
// 安全：①②③ 全程 ARM_2 任一关节离开 hold 位置超过它就整机停 [rad]，0.01 rad ≈ 0.6°，同 arm1_transparent_arm2_hold_rt.cpp。
// 结束时打印 ③ 期间 ARM_2 最大偏离和速度。
constexpr double kArm2MaxPosError = 0.01;
constexpr auto kHeartbeatTimeout = std::chrono::milliseconds(100);  // 安全：③ 实时任务这么久没成功发出指令就整机停。
constexpr double kTimelinessFailureLimit = 2.0;  // 安全：SetTimelinessFailureLimit，1 秒内迟到指令的百分比上限，RDK 默认 2。
constexpr uint64_t kWarmupCycles = 100;  // 流程：③ 先用名义刚度成功发满这么多次指令，才降刚度。
constexpr auto kWarmupTimeout = std::chrono::seconds(2);  // 流程：发满 kWarmupCycles 次的最长时间。
constexpr double kOperationalTimeout = 30.0;  // 流程：上伺服后等待就绪的最长时间 [s]。
constexpr double kHomeTimeout = 30.0;  // 流程：① 回正到位并停稳的最长时间 [s]。
constexpr double kZeroTimeout = 10.0;  // 流程：② 受力回 0 结束的最长时间 [s]。
// 流程：主线程检查断线、E-stop、故障、控制模式、心跳的周期；--log 也按这个周期记一行（100 Hz）。
constexpr auto kMonitorPeriod = std::chrono::milliseconds(10);

struct Params  // 命令行覆盖之后实际使用的参数。
{
    std::string robot_sn;
    double stiffness_scale = kStiffnessScale;
    double null_space_tracking = kNullSpaceRefTracking;
    bool passive = kPassiveForceControl;
    std::string log_path;  // 空 = 不记录。
    bool home = true;
    bool clear_fault = false;
};

struct Arm2Record  // ③ 期间 ARM_2 最大偏离 hold 位置 [rad] 和最大速度 [rad/s]：实时任务写，调度器停下后主线程读。
{
    std::array<double, kDoF> max_error = {};
    std::array<double, kDoF> max_vel = {};
};

struct RtTargets  // ③ 的指令和检查用的数据：进入 ③ 时定好，调度器启动后实时任务只读。
{
    std::map<rdk::JointGroup, rdk::RtCartesianCmd> commands;  // 每 1 ms 发出的两条臂指令，一直不变。
    std::array<double, rdk::kPoseSize> arm1_pose = {};  // ARM_1 进入 ③ 时的末端位姿：目标位姿，也是 kMaxTcpDisplacement 的起点。
    std::vector<double> arm1_q_min, arm1_q_max, arm1_dq_max;  // ARM_1 关节位置极限 [rad] 和速度极限 [rad/s]。
    std::vector<double> arm2_q;  // ARM_2 hold 时的关节位置 [rad]，只用来检查 ARM_2 动没动。
};

std::atomic<bool> g_stop = {false};  // 置 true 后实时任务不再发指令，主线程收尾。信号处理函数只做这一件事。
std::mutex g_reason_mutex;  // 保护 g_reason：实时线程和主线程都可能写。
std::string g_reason;  // 第一个停机原因；空 = Ctrl+C 正常结束。
std::atomic<uint64_t> g_sent_count = {0};  // ③ 实时任务成功发出指令的次数。
std::atomic<int64_t> g_last_sent_ns = {0};  // ③ 实时任务最近一次成功发出指令的时刻 [ns]，见 NowNs。

void PrintHelp()
{
    std::cout << "用法：sudo .build/script/intermediate8_realtime_cartesian_floating <序列号> [选项]\n"
                 "  <序列号>          完整序列号，不要有空格，例如 Enlight-LL-123456\n"
                 "  --scale=S         覆盖 kStiffnessScale：ARM_1 六轴刚度占名义刚度的比例 [0, 1]，0 = 零刚度浮动\n"
                 "  --nullspace=W     覆盖 kNullSpaceRefTracking：肘部拉回参考姿态的权重 [0.1, 1]\n"
                 "  --passive         覆盖 kPassiveForceControl：六轴被动零力控制，需要 6 维力传感器\n"
                 "  --log=FILE.csv    ③ 浮动期间按 100 Hz 记录 ARM_1 的末端位姿、速度、外力和关节位置、速度\n"
                 "  --no-home         不先回正，在当前姿态受力回 0 后直接浮动\n"
                 "  --clear-fault     启动时若有故障先尝试清除；不加则直接退出\n"
                 "  -h, --help        显示本说明\n";
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

bool OptionValue(const std::string& arg, const std::string& name, std::string& value)  // arg 是 "name=值" 时取出值。
{
    if (!arg.starts_with(name + "=")) {
        return false;
    }
    value = arg.substr(name.size() + 1);
    return true;
}

std::string ParseArgs(int argc, char* argv[], Params& params)  // 读取命令行；未给出的用上方常量。出错返回原因。
{
    params.robot_sn = argv[1];
    if (params.robot_sn.starts_with("-")) {
        return "第一个参数必须是序列号";
    }
    for (int i = 2; i < argc; ++i) {
        const std::string arg = argv[i];
        std::string value;
        if (OptionValue(arg, "--scale", value)) {
            if (!ParseNumber(value.c_str(), params.stiffness_scale)) {
                return "--scale= 后面要跟 1 个数";
            }
        } else if (OptionValue(arg, "--nullspace", value)) {
            if (!ParseNumber(value.c_str(), params.null_space_tracking)) {
                return "--nullspace= 后面要跟 1 个数";
            }
        } else if (OptionValue(arg, "--log", value)) {
            if (value.empty()) {
                return "--log= 后面要跟文件名";
            }
            params.log_path = value;
        } else if (arg == "--passive") {
            params.passive = true;
        } else if (arg == "--no-home") {
            params.home = false;
        } else if (arg == "--clear-fault") {
            params.clear_fault = true;
        } else {
            return "不认识的参数 " + arg;
        }
    }
    // 下发前按 RDK 文档范围检查；上方常量也一样要过这一关。刚度上限 K_x_nom 由比例 ≤ 1 保证。
    if (!(params.stiffness_scale >= 0.0 && params.stiffness_scale <= 1.0)) {
        return "stiffness_scale = " + Num(params.stiffness_scale) + " 超出范围 [0, 1]";
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

// 绝对值超过 limit 的关节，例如 "J2 1.05、J4 1.10"；都没超过返回空串。写成 !(… <= limit)：读数是 NaN 也算超过。
template <typename Values>
std::string OverLimit(const Values& values, double limit, int decimals)
{
    bool over = false;
    for (size_t i = 0; i < kDoF; ++i) {
        over = over || !(std::abs(values[i]) <= limit);
    }
    if (!over) {
        return "";  // 每 1 ms 都走这里，不分配内存。
    }
    std::ostringstream oss;
    oss.precision(decimals);
    oss << std::fixed;
    for (size_t i = 0; i < kDoF; ++i) {
        if (!(std::abs(values[i]) <= limit)) {
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

double Norm3(const std::array<double, kCartDoF>& v, size_t offset)  // v[offset]、v[offset+1]、v[offset+2] 组成的向量长度。
{
    return std::sqrt(v[offset] * v[offset] + v[offset + 1] * v[offset + 1] + v[offset + 2] * v[offset + 2]);
}

int64_t NowNs()  // steady_clock 的当前时刻 [ns]：实时线程记录、主线程比较，用来检查心跳。
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

// ③ ARM_1 的检查：末端速度、末端离起点距离、关节离位置极限的余量、关节速度，任一不满足就抛出。
// 比较都写成 !(值 <= 上限)：读数是 NaN 也算不满足。
void CheckArm1(const rdk::RobotStates& arm1, const RtTargets& targets)
{
    const double linear_vel = Norm3(arm1.tcp_twist, 0);
    const double angular_vel = Norm3(arm1.tcp_twist, 3);
    if (!(linear_vel <= kMaxTcpLinearVel) || !(angular_vel <= kMaxTcpAngularVel)) {
        throw std::runtime_error("ARM_1 末端速度 " + Num(linear_vel) + " m/s、" + Num(angular_vel)
                                 + " rad/s，超过 kMaxTcpLinearVel = " + Num(kMaxTcpLinearVel)
                                 + " m/s 或 kMaxTcpAngularVel = " + Num(kMaxTcpAngularVel) + " rad/s");
    }
    double distance2 = 0.0;
    for (size_t i = 0; i < 3; ++i) {
        const double d = arm1.tcp_pose[i] - targets.arm1_pose[i];
        distance2 += d * d;
    }
    if (!(distance2 <= kMaxTcpDisplacement * kMaxTcpDisplacement)) {
        throw std::runtime_error("ARM_1 末端离起点 " + Num(std::sqrt(distance2)) + " m，超过 kMaxTcpDisplacement = "
                                 + Num(kMaxTcpDisplacement) + " m");
    }
    for (size_t i = 0; i < kDoF; ++i) {
        const double q_min = targets.arm1_q_min[i], q_max = targets.arm1_q_max[i];
        if (!(arm1.q[i] >= q_min + kJointLimitMargin && arm1.q[i] <= q_max - kJointLimitMargin)) {
            throw std::runtime_error("ARM_1 J" + std::to_string(i + 1) + " = " + Num(arm1.q[i]) + " rad，离位置极限 ["
                                     + Num(q_min) + ", " + Num(q_max) + "] 不到 kJointLimitMargin = "
                                     + Num(kJointLimitMargin) + " rad");
        }
        const double vel_limit = kJointVelFraction * targets.arm1_dq_max[i];
        if (!(std::abs(arm1.dtheta[i]) <= vel_limit)) {  // 电机侧速度，噪声小。
            throw std::runtime_error("ARM_1 J" + std::to_string(i + 1) + " 速度 " + Num(std::abs(arm1.dtheta[i]))
                                     + " rad/s，超过 dq_max × kJointVelFraction = " + Num(vel_limit) + " rad/s");
        }
    }
}

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
        CheckArm1(arm1, targets);

        // ARM_1：进入 ③ 时的位姿、目标力 0。ARM_2：进入 ③ 时的位姿，就是 hold。两条臂每次都一起发。
        robot.StreamCartesianMotionForce(targets.commands);
        g_last_sent_ns = NowNs();
        ++g_sent_count;
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

std::string HeartbeatLost()  // ③ 实时任务超过 kHeartbeatTimeout 没有成功发出指令的原因；正常返回空串。
{
    const auto age = std::chrono::nanoseconds(NowNs() - g_last_sent_ns);
    if (age <= kHeartbeatTimeout) {
        return "";
    }
    return "实时任务 " + Num(std::chrono::duration<double, std::milli>(age).count()) + " ms 没有发出指令（kHeartbeatTimeout = "
           + Num(static_cast<double>(kHeartbeatTimeout.count())) + " ms）";
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
    if (params.passive) {  // 上伺服、回正之前就检查，免得回正完才被拒绝。RDK：没装力传感器时 raw_ft_sensor 为 0。
        const auto ft = robot.states().at(rdk::JointGroup::ARM_1).raw_ft_sensor;
        if (std::all_of(ft.begin(), ft.end(), [](double v) { return v == 0.0; })) {
            throw std::runtime_error(
                "--passive 要 ARM_1 有 6 维力传感器（raw_ft_sensor 全为 0，没装）；改用 --scale=0 做六维零刚度浮动");
        }
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

// ③ 降刚度之前：等实时任务用名义刚度成功发满 kWarmupCycles 次指令。被 Ctrl+C 打断或实时任务已要求停机时返回 false。
bool WaitWarmup(const rdk::Robot& robot)
{
    const auto deadline = std::chrono::steady_clock::now() + kWarmupTimeout;
    while (g_sent_count < kWarmupCycles) {
        if (g_stop) {
            return false;
        }
        if (const auto reason = NotRunningReason(robot, rdk::Mode::RT_CARTESIAN_MOTION_FORCE); !reason.empty()) {
            throw std::runtime_error(reason);
        }
        if (std::chrono::steady_clock::now() > deadline) {
            throw std::runtime_error("实时任务 " + Num(std::chrono::duration<double>(kWarmupTimeout).count())
                                     + " 秒内没有成功发满 " + std::to_string(kWarmupCycles) + " 次指令");
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return !g_stop;
}

void WriteLogHeader(std::ostream& out)  // --log 的表头：时间、ARM_1 末端位姿、速度、外力，关节位置、速度。
{
    out << "t_s,x,y,z,qw,qx,qy,qz,vx,vy,vz,wx,wy,wz,fx,fy,fz,mx,my,mz";
    for (const char* name : {"q", "dtheta"}) {
        for (size_t i = 1; i <= kDoF; ++i) {
            out << ',' << name << i;
        }
    }
    out << '\n';
}

// --log 的一行，t 是开始浮动后的秒数。末端位姿、速度、外力都在世界坐标系；外力 tcp_wrench 是 ② 清零之后的读数。
void WriteLogRow(std::ostream& out, double t, const rdk::RobotStates& arm1)
{
    out << t;
    const auto write = [&out](const auto& values) {
        for (double v : values) {
            out << ',' << v;
        }
    };
    write(arm1.tcp_pose);
    write(arm1.tcp_twist);
    write(arm1.tcp_wrench);
    write(arm1.q);
    write(arm1.dtheta);
    out << '\n';
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
    std::ofstream log;  // 连机器人之前打开：路径不对时机器人还没动就退出。
    if (!params.log_path.empty()) {
        log.open(params.log_path);
        if (!log) {
            std::cerr << "打不开日志文件 " << params.log_path << "\n";
            return 2;
        }
        log.precision(9);
    }

    // 声明顺序决定析构顺序：调度器最先析构，实时任务不会用到已经销毁的 robot 和 targets。
    std::unique_ptr<rdk::Robot> robot;
    RtTargets targets;
    Arm2Record arm2_record;
    std::unique_ptr<rdk::Scheduler> scheduler;
    bool need_stop = false, started = false, logging = false;
    try {
        scheduler = std::make_unique<rdk::Scheduler>();  // 先建调度器：系统问题在连机器人之前就暴露。
        robot = std::make_unique<rdk::Robot>(params.robot_sn);
        for (int sig : {SIGINT, SIGTERM, SIGHUP}) {  // 在 RDK 初始化之后装，保证由这里接管信号。
            std::signal(sig, OnSignal);
        }
        Prepare(*robot, params);
        const auto info = robot->info();
        const auto& arm1_K_nom = info.K_x_nom.at(rdk::JointGroup::ARM_1);
        auto arm1_float_K = arm1_K_nom;  // ARM_1 浮动时的六轴刚度 = K_x_nom × stiffness_scale。
        for (auto& k : arm1_float_K) {
            k *= params.stiffness_scale;
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
        std::cout << "② ARM_1 受力回 0 中（ZeroFTSensor），ARM_2 Hold。不要碰 ARM_1 ……" << std::endl;
        PrintArm1Force(*robot, "前");
        RunArm1Primitive(*robot,
            rdk::PrimitiveArgs("ZeroFTSensor", {{"dataCollectTime", kZeroDataCollectTime},
                                                   {"enableStaticCheck", kZeroEnableStaticCheck},
                                                   {"calibExtraPayload", kZeroCalibExtraPayload}}),
            arm2_start, kZeroTimeout, [](const rdk::PrimitiveStates* state) {
                return !state || rdk::utility::PrimitiveStateTrue(*state, "terminated");  // 文档默认转移条件。
            });
        PrintArm1Force(*robot, "后");

        // ③ 从这里到结束全程实时模式，不再切换。照官方 intermediate5/6 的顺序：先切模式，再设置，再读起始位姿。
        robot->SetTimelinessFailureLimit(kTimelinessFailureLimit);
        robot->SwitchMode(rdk::Mode::RT_CARTESIAN_MOTION_FORCE);
        for (const auto group : {rdk::JointGroup::ARM_1, rdk::JointGroup::ARM_2}) {  // 两条臂先都是 6 轴纯位置控制、名义刚度。
            robot->SetForceControlAxis(group, {false, false, false, false, false, false});
            robot->SetCartesianImpedance(group, info.K_x_nom.at(group));
        }
        robot->SetForceControlFrame(rdk::JointGroup::ARM_1, rdk::CoordType::WORLD);  // RDK 默认；只在 --passive 开力控时用到。
        robot->SetNullSpaceObjectives(rdk::JointGroup::ARM_1, 0.0, 0.0, params.null_space_tracking);
        const auto states = robot->states();
        const auto& arm1 = states.at(rdk::JointGroup::ARM_1);
        const auto& arm2 = states.at(rdk::JointGroup::ARM_2);
        robot->SetNullSpacePosture(rdk::JointGroup::ARM_1, arm1.q);  // 肘部参考姿态 = 进入 ③ 时的姿态。
        targets.arm1_pose = arm1.tcp_pose;
        targets.arm1_q_min = info.q_min.at(rdk::JointGroup::ARM_1);
        targets.arm1_q_max = info.q_max.at(rdk::JointGroup::ARM_1);
        targets.arm1_dq_max = info.dq_max.at(rdk::JointGroup::ARM_1);
        targets.arm2_q = arm2.q;
        targets.commands = {
            {rdk::JointGroup::ARM_1, rdk::RtCartesianCmd(arm1.tcp_pose, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0})},
            {rdk::JointGroup::ARM_2, rdk::RtCartesianCmd(arm2.tcp_pose)},  // 只给位姿，就是纯位置控制。
        };
        CheckArm1(arm1, targets);  // 进入 ③ 时就不满足（例如关节离位置极限太近）就不开始。
        std::cout << "ARM_1：名义刚度 K_x_nom [" << rdk::utility::Arr2Str(arm1_K_nom, 1) << "] N/m、Nm/rad；浮动时"
                  << (params.passive ? std::string("六轴被动零力控制（kStiffnessScale 不起作用）")
                                     : "刚度 × " + Num(params.stiffness_scale) + " = ["
                                           + rdk::utility::Arr2Str(arm1_float_K, 1) + "]")
                  << "；肘部拉回权重 " << Num(params.null_space_tracking) << std::endl;
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
        std::cout << "③ 实时模式 RT_CARTESIAN_MOTION_FORCE 已开始：ARM_2 hold 不动；ARM_1 先用名义刚度保持，扶住 ARM_1 ……"
                  << std::endl;

        if (WaitWarmup(*robot) && !g_stop) {
            // 切到浮动：实时模式里的参数设置，不切换控制模式；实时任务照常每 1 ms 给两条臂发指令。
            if (params.passive) {
                robot->SetForceControlAxis(rdk::JointGroup::ARM_1, {true, true, true, true, true, true});  // 目标力 0。
            } else {
                robot->SetCartesianImpedance(rdk::JointGroup::ARM_1, arm1_float_K);
            }
            std::cout << "ARM_1 已浮动，可以手拖；ARM_2 hold 不动。按 Ctrl+C 结束。" << std::endl;
            if (log.is_open()) {
                WriteLogHeader(log);
                logging = true;
            }
        }
        const auto float_start = std::chrono::steady_clock::now();
        while (!g_stop) {
            auto reason = NotRunningReason(*robot, rdk::Mode::RT_CARTESIAN_MOTION_FORCE);
            if (reason.empty()) {
                reason = HeartbeatLost();
            }
            if (!reason.empty()) {
                RequestStop(reason);
                break;
            }
            if (logging) {  // 文件读写在主线程，不在实时任务里。
                WriteLogRow(log, std::chrono::duration<double>(std::chrono::steady_clock::now() - float_start).count(),
                    robot->states().at(rdk::JointGroup::ARM_1));
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
    if (logging) {
        log.close();
        std::cout << "日志已写入 " << params.log_path << std::endl;
    }

    std::lock_guard<std::mutex> lock(g_reason_mutex);
    if (!g_reason.empty()) {
        std::cerr << "已停止：" << g_reason << std::endl;
        return 1;
    }
    return 0;
}
