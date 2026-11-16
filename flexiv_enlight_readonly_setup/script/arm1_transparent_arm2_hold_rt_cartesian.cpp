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
// ARM_1 软停止（arm1_soft_stop.hpp，说明见 README_arm1_soft_stop.md）：③ 期间的普通 Stop Sign（终端按 Enter、
// sudo kill -USR1 <pid>、--stop-di 的数字输入变高）只让 ARM_1 停下：实时任务锁存一次实测位姿和速度，按二次位置曲线
// 减速到 0，停稳后 HOLD 在终点；ARM_2 的 hold 指令不变，两条臂照旧每 1 ms 一起发，不调用 Stop()、不退出实时模式。
// 透明时 X Y Z 是力控、转动刚度 0，位姿指令不起作用，所以锁存后主线程调一次 SetCartesianImpedance、SetForceControlAxis
// 把 ARM_1 切回运动控制（阻塞调用，不在实时回调里）。Stop Sign 不是急停：E-stop、控制器自带的保护、下面的停机检查都照旧，
// 不经过软停止。SOFT_STOP、HOLD 不会自己回到透明，要恢复就 Ctrl+C 结束后重新运行。
//
// 编译（项目根目录）：cmake -S script -B .build/script && cmake --build .build/script
// 运行（实时调度器需要 root）：sudo .build/script/arm1_transparent_arm2_hold_rt_cartesian <序列号> [选项]；--help 看选项。

#include <flexiv/rdk/robot.hpp>
#include <flexiv/rdk/scheduler.hpp>
#include <flexiv/rdk/utility.hpp>

#include "arm1_soft_stop.hpp"  // ARM_1 软停止的状态机、轨迹、日志，和离线单元测试 arm1_soft_stop_test.cpp 共用。

#include <poll.h>  // poll：不阻塞地查终端有没有按 Enter。
#include <termios.h>  // tcflush：丢掉 ③ 之前在终端按的键。
#include <unistd.h>  // geteuid：检查是否以 root 运行；read、getpid：终端 Stop Sign、打印 SIGUSR1 要用的进程号。
// 输出用 std::cout 而不是 spdlog：/usr/local/include 里的 fmt 10 会盖住系统 spdlog 1.9 需要的 fmt 8，编不过。

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
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

// ================================ ARM_1 软停止：在这里调 ================================
// 普通 Stop Sign 的来源：终端按 Enter；sudo kill -USR1 <pid>（给视觉等别的程序用）；--stop-di N 时数字输入 DI N 变高。
//   都在主线程每 kMonitorPeriod 查一次，只把 g_stop_sign_count 加 1，不直接动机器人；实时任务看到后锁存 ARM_1 软停止。
// 下面的值是项目自定的研发参数，不是厂家安全认证值；第一次上真机用小速度，见 README_arm1_soft_stop.md 的 Test B、C。
// T = max(kStopMinTime, |v0| / a_max, |ω0| / α_max)。速度再大也照样软停止，不设停止距离、转角、T 的上限：
//   停止距离 = |v0|² / (2 a_max)，0.5 m/s² 时 0.2 m/s 走 4 cm，0.5 m/s 走 25 cm，1 m/s 走 50 cm。要停得更快就调大 a_max。
constexpr double kStopMaxLinearDecel = 0.5;  // a_max：末端线减速度上限 [m/s²]
constexpr double kStopMaxAngularDecel = 1.0;  // α_max：角减速度上限 [rad/s²]
constexpr double kStopMinTime = 0.3;  // T 的下限 [s]：速度很小时也用这么久减速。
constexpr size_t kTwistFilterSamples = 5;  // v0、ω0 取最近几个周期 tcp_twist 的平均：1 ms 一个，延迟约 2 ms。
// 实测状态（RobotStates::timestamp）连续这么多个周期没变算过期：不开始软停止；SOFT_STOP、HOLD 中就走原有停机。
constexpr int kMaxStateStaleTicks = 20;
// SOFT_STOP → HOLD：t ≥ T 之后下面五项连续满足 kHoldSettleTime 才进 HOLD；T 之后 kHoldTimeout 秒还不满足就走原有停机。
constexpr double kHoldLinearVel = 0.005;  // 实测末端线速度 [m/s]
constexpr double kHoldAngularVel = 0.02;  // 实测角速度 [rad/s]
constexpr double kHoldPositionTol = 0.005;  // 离终点位置 [m]
constexpr double kHoldAngleTol = 0.03;  // 离终点姿态 [rad]
constexpr double kHoldMaxForce = 20.0;  // 末端外力 tcp_wrench 的 |f| [N]：大说明有人推或碰到东西。
constexpr double kHoldSettleTime = 0.2;  // [s]
constexpr double kHoldTimeout = 3.0;  // [s]
// SOFT_STOP、HOLD 中实测离目标超过下面的值就走原有停机（被推开、碰到东西、没切到运动控制），不硬拉回去。
constexpr double kMaxTrackPositionError = 0.05;  // [m]
constexpr double kMaxTrackAngleError = 0.3;  // [rad]
// kStopStiffnessScale：锁存后 ARM_1 的笛卡尔刚度 = K_x_nom × 它（不低于透明时的刚度；锁住的 X Y Z 保持 K_x_nom），
//   同时 X Y Z 关力控。范围 (0, 1]，1 就是 ARM_2 hold 用的名义刚度。命令行 --stop-stiffness-scale。
constexpr double kStopStiffnessScale = 1.0;
// kStiffnessRamp：第二阶段的刚度渐增，默认关。开了之后刚度按 K_x(t) = K0 + (Kf - K0)(t/T)² 从透明时的实际刚度
//   （力控的 X Y Z 按 0）涨到上面的停止刚度：主线程每 kStiffnessRampPeriod 调一次阻塞的 SetCartesianImpedance，
//   实际刚度是阶梯。刚度从 0 起步时 ARM_1 一开始几乎不跟踪减速轨迹（阻尼也是 0），只在 Test D 里用。命令行 --stiffness-ramp。
constexpr bool kStiffnessRamp = false;
constexpr auto kStiffnessRampPeriod = std::chrono::milliseconds(50);
constexpr size_t kLogQueueRows = 8192;  // --log：实时任务到主线程的日志队列，1 kHz 一行，约 8 秒；满了丢行并计数。
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
constexpr double kLateCycleMs = 2.0;  // 统计：实时周期间隔超过它算一次迟到，结束时打印次数，不停机 [ms]。
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
    int stop_di = -1;  // Stop Sign 用的数字输入口，-1 = 不用。
    double stop_stiffness_scale = kStopStiffnessScale;
    bool stiffness_ramp = kStiffnessRamp;
    std::string log_path;  // 空 = 不记录。
};

struct Arm2Record  // ③ 期间 ARM_2 最大偏离 hold 位置 [rad] 和最大速度 [rad/s]：实时任务写，调度器停下后主线程读。
{
    std::array<double, kDoF> max_error = {};
    std::array<double, kDoF> max_vel = {};
};

std::atomic<bool> g_stop = {false};  // 置 true 后实时任务不再发指令，主线程收尾。信号处理函数只做这一件事。
std::mutex g_reason_mutex;  // 保护 g_reason：实时线程和主线程都可能写。
std::string g_reason;  // 第一个停机原因；空 = Ctrl+C 正常结束。
std::atomic<uint64_t> g_sigusr1_count = {0};  // SIGUSR1 的信号处理函数只把它加 1。
std::atomic<uint64_t> g_stop_sign_count = {0};  // 主线程每收到一次 Stop Sign 加 1；实时任务看到大于 0 就锁存 ARM_1 软停止。
// 实时任务每周期发布的 ARM_1 状态（soft_stop::Arm1State）。主线程看到它离开 TRANSPARENT 之后才读软停止的 plan。
std::atomic<int> g_arm1_state = {0};
std::atomic<bool> g_arm1_motion_ctrl = {false};  // 主线程已把 ARM_1 切回运动控制；只用于日志。
std::array<std::atomic<double>, kCartDoF> g_arm1_set_K;  // 主线程最近一次 SetCartesianImpedance(ARM_1) 完成的刚度；只用于日志。
std::atomic<double> g_impedance_latency_ms = {soft_stop::kNaN};  // 那一次调用的用时；只用于日志。
std::atomic<uint64_t> g_log_dropped = {0};  // 日志队列满了丢掉的行数。

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
                 "  --stop-di N                 数字输入 DI N（0–19）变高也算 Stop Sign；不加只用终端 Enter 和 SIGUSR1\n"
                 "  --stop-stiffness-scale S    覆盖 kStopStiffnessScale：软停止后 ARM_1 刚度占 K_x_nom 的比例，范围 (0, 1]\n"
                 "  --stiffness-ramp            覆盖 kStiffnessRamp：软停止时刚度按 (t/T)² 渐增（第二阶段实验，默认关）\n"
                 "  --log FILE.csv              ③ 期间每个实时周期记一行（约 1 MB/s）；FILE.csv 已经存在时不运行\n"
                 "  -h, --help                  显示本说明\n"
                 "③ 期间终端按 Enter 或 sudo kill -USR1 <pid> 发普通 Stop Sign：ARM_1 软停止后 HOLD，ARM_2 照旧 hold；\n"
                 "Ctrl+C 结束程序（整机 Stop()）。Stop Sign 不是急停，急停用 E-stop。\n";
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

soft_stop::StopConfig MakeStopConfig(const Params& params)  // 上方软停止常量和命令行的力控轴，给 ARM_1 软停止状态机。
{
    soft_stop::StopConfig c;
    // 力控的 X Y Z 透明时能拖，软停止从实测速度减速；位置控制的轴保持原来的目标。力控坐标系是 TCP 时锁住的方向
    // 不是世界系的轴，就都按能拖处理（从实测位姿开始，那几个轴本来就几乎不动）。
    c.free_axes = kForceCtrlFrame == rdk::CoordType::WORLD ? params.force_axis : std::array<bool, 3> {true, true, true};
    c.max_linear_decel = kStopMaxLinearDecel;
    c.max_angular_decel = kStopMaxAngularDecel;
    c.min_time = kStopMinTime;
    c.max_stale_ticks = kMaxStateStaleTicks;
    c.hold_linear_vel = kHoldLinearVel;
    c.hold_angular_vel = kHoldAngularVel;
    c.hold_position_tol = kHoldPositionTol;
    c.hold_angle_tol = kHoldAngleTol;
    c.hold_max_force = kHoldMaxForce;
    c.hold_settle_time = kHoldSettleTime;
    c.hold_timeout = kHoldTimeout;
    c.max_track_position_error = kMaxTrackPositionError;
    c.max_track_angle_error = kMaxTrackAngleError;
    return c;
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
        } else if (arg == "--stop-di") {
            double value = 0.0;
            if (++i >= argc || !ParseNumber(argv[i], value) || value != std::floor(value) || value < 0.0
                || value >= static_cast<double>(rdk::kIOPorts)) {
                return "--stop-di 后面要跟 1 个 0 到 " + std::to_string(rdk::kIOPorts - 1) + " 的整数";
            }
            params.stop_di = static_cast<int>(value);
        } else if (arg == "--stop-stiffness-scale") {
            if (++i >= argc || !ParseNumber(argv[i], params.stop_stiffness_scale)) {
                return "--stop-stiffness-scale 后面要跟 1 个数";
            }
        } else if (arg == "--stiffness-ramp") {
            params.stiffness_ramp = true;
        } else if (arg == "--log") {
            if (++i >= argc) {
                return "--log 后面要跟文件名";
            }
            params.log_path = argv[i];
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
    if (!(params.stop_stiffness_scale > 0.0 && params.stop_stiffness_scale <= 1.0)) {  // 0 = 没有刚度，减速轨迹不起作用。
        return "stop_stiffness_scale = " + Num(params.stop_stiffness_scale) + " 超出范围 (0, 1]";
    }
    if (const auto error = soft_stop::CheckConfig(MakeStopConfig(params)); !error.empty()) {
        return "软停止参数：" + error;
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

void OnStopSignal(int)  // SIGUSR1：别的程序（例如视觉）发来的普通 Stop Sign。只计数，主线程去处理。
{
    ++g_sigusr1_count;
}

double SteadySeconds()  // 单调时钟的当前时刻 [s]：实时任务算软停止的 t，主线程算刚度渐增的 t，用同一个时钟。
{
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
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

struct RtTargets  // ③ 的指令和 ARM_1 软停止的状态：进入 ③ 时定好，调度器启动后只有实时任务改。
{
    // 每 1 ms 发出的两条臂指令。ARM_2 那条进入 ③ 时定好（hold 位姿，wrench、twist、acc 都是 0），之后不再改；
    // ARM_1 那条每周期由 arm1 状态机算：透明时和原来一样，力控轴目标力 0，位置控制轴锁在进入 ③ 时的位姿。
    std::map<rdk::JointGroup, rdk::RtCartesianCmd> commands;
    std::array<double, rdk::kPoseSize> arm2_pose = {};  // ARM_2 hold 的末端位姿，只用来记录末端偏离。
    std::vector<double> arm2_q;  // ARM_2 hold 时的关节位置 [rad]（只用来检查 ARM_2 动没动）。
    std::optional<soft_stop::Arm1StopMachine> arm1;  // ARM_1 的 TRANSPARENT → SOFT_STOP → HOLD。
    soft_stop::TwistFilter twist_filter {kTwistFilterSamples};
    soft_stop::StaleCounter stale;
    soft_stop::Vec6 K0 = {};  // 只用于日志：透明时的实际刚度、停止刚度、是否渐增，同 Arm1StopStiffness。
    soft_stop::Vec6 K_stop = {};
    bool stiffness_ramp = false;
    double start_time = 0.0;  // ③ 开始的时刻 [s]，见 SteadySeconds。
    double last_tick = soft_stop::kNaN;  // 上一个实时周期的时刻 [s]
    double max_cycle_ms = 0.0;  // 最长的实时周期间隔。
    uint64_t late_cycles = 0;  // 间隔超过 kLateCycleMs 的周期数。
    soft_stop::SpscRing<soft_stop::LogRow>* log = nullptr;  // --log 的队列；不记录时为空。
    soft_stop::LogRow row;  // 这个周期的日志行。
};

// --log 的一行：这个周期的实测值、发给 ARM_1 的指令、刚度。只拷贝固定大小的数据，不分配内存。
void FillLogRow(RtTargets& targets, double now, uint64_t stop_signs, const rdk::RobotStates& arm1,
    const rdk::RobotStates& arm2, const std::array<double, kDoF>& arm2_error, const soft_stop::Measurement& measured,
    const soft_stop::CartesianCmd& arm1_cmd)
{
    auto& row = targets.row;
    const auto& machine = *targets.arm1;
    const auto state = machine.state();
    row.stop_sign = static_cast<double>(stop_signs);
    row.arm1_state = static_cast<double>(state);
    if (state != soft_stop::Arm1State::TRANSPARENT) {
        row.stop_elapsed_s = now - machine.plan().t0;
        row.stop_T_s = machine.plan().T;
    }
    row.arm1_pose = arm1.tcp_pose;
    row.arm1_twist = arm1.tcp_twist;
    std::copy_n(arm1.tcp_wrench.begin(), 3, row.arm1_force.begin());
    row.target_pose = arm1_cmd.pose_d;
    row.target_twist = arm1_cmd.twist_d;
    row.target_acc = arm1_cmd.acc_d;
    std::copy_n(arm1.q.begin(), kDoF, row.arm1_q.begin());
    std::copy_n(arm1.dtheta.begin(), kDoF, row.arm1_dtheta.begin());
    std::copy_n(arm2.tcp_pose.begin(), 3, row.arm2_position.begin());
    std::copy_n(arm2.q.begin(), kDoF, row.arm2_q.begin());
    std::copy_n(arm2.dtheta.begin(), kDoF, row.arm2_dtheta.begin());
    row.arm2_hold_position_error = soft_stop::Distance(arm2.tcp_pose, targets.arm2_pose);
    row.arm2_hold_joint_error = 0.0;
    for (double e : arm2_error) {
        row.arm2_hold_joint_error = std::max(row.arm2_hold_joint_error, std::abs(e));
    }
    row.requested_Kx = state == soft_stop::Arm1State::TRANSPARENT
                           ? targets.K0
                           : soft_stop::StopStiffness(
                               targets.stiffness_ramp, targets.K0, targets.K_stop, row.stop_elapsed_s, row.stop_T_s);
    for (size_t i = 0; i < kCartDoF; ++i) {
        row.set_Kx[i] = g_arm1_set_K[i].load(std::memory_order_relaxed);
    }
    row.impedance_latency_ms = g_impedance_latency_ms.load(std::memory_order_relaxed);
    row.motion_ctrl = g_arm1_motion_ctrl ? 1.0 : 0.0;
    row.state_stale_ticks = measured.stale_ticks;
    row.fault_status = 0.0;
}

// 实时任务：每 1 ms 检查一次，算出 ARM_1 这一周期的指令（透明 / 软停止 / HOLD），然后把两条臂的末端指令放进
// 同一条 StreamCartesianMotionForce 发出去。ARM_2 那条指令进入 ③ 时就定好，这里不改。
void PeriodicTask(rdk::Robot& robot, RtTargets& targets, Arm2Record& record)
{
    if (g_stop) {
        return;  // 要停了：不再发指令，等主线程停掉调度器后 Stop()。
    }
    const double now = SteadySeconds();
    auto& row = targets.row;
    if (targets.log) {
        row = soft_stop::LogRow {};
        row.time_s = now - targets.start_time;
    }
    if (!std::isnan(targets.last_tick)) {  // 实时周期间隔：只统计，不停机。
        const double cycle_ms = (now - targets.last_tick) * 1000.0;
        targets.max_cycle_ms = std::max(targets.max_cycle_ms, cycle_ms);
        targets.late_cycles += cycle_ms > kLateCycleMs ? 1 : 0;
        row.rt_cycle_us = cycle_ms * 1000.0;
    }
    targets.last_tick = now;
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

        // ARM_1：没有 Stop Sign 时发原来的透明指令；收到 Stop Sign 的周期锁存一次实测位姿和速度，之后按二次位置曲线
        // 减速，停稳后 HOLD；速度再大也照样执行。实测值无效、没停稳、跟踪误差太大时抛出，走原有的停机。
        soft_stop::Measurement measured;
        measured.tcp_pose = arm1.tcp_pose;
        measured.tcp_twist = targets.twist_filter.Update(arm1.tcp_twist);
        measured.tcp_wrench = arm1.tcp_wrench;
        measured.stale_ticks = targets.stale.Update(arm1.timestamp.first, arm1.timestamp.second);
        const uint64_t stop_signs = g_stop_sign_count;
        soft_stop::CartesianCmd arm1_cmd;
        if (const auto error = targets.arm1->Step(now, stop_signs > 0, measured, arm1_cmd); !error.empty()) {
            throw std::runtime_error("ARM_1 软停止：" + error);
        }
        g_arm1_state.store(static_cast<int>(targets.arm1->state()), std::memory_order_release);
        auto& cmd = targets.commands.at(rdk::JointGroup::ARM_1);
        cmd.pose_d = arm1_cmd.pose_d;
        cmd.wrench_d = arm1_cmd.wrench_d;
        cmd.twist_d = arm1_cmd.twist_d;
        cmd.acc_d = arm1_cmd.acc_d;
        if (targets.log) {
            FillLogRow(targets, now, stop_signs, arm1, arm2, arm2_error, measured, arm1_cmd);
        }

        // ARM_1 和 ARM_2 一起发。ARM_2：只给位姿，就是纯位置控制（官方 intermediate5 的写法）。
        robot.StreamCartesianMotionForce(targets.commands);
        row.stream_status = 1.0;
    } catch (const std::exception& e) {
        row.stream_status = 0.0;
        RequestStop(e.what());
    }
    if (targets.log && !targets.log->TryPush(row)) {
        ++g_log_dropped;
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

// 普通 Stop Sign 的三个来源，都在主线程每 kMonitorPeriod 查一次。查到了只返回来源，由调用方把 g_stop_sign_count 加 1；
// 这里不动机器人。
struct StopSignInput
{
    bool keyboard = true;  // 终端按 Enter；stdin 关了（EOF）就不再读。
    uint64_t usr1_seen = 0;  // 已经处理过的 SIGUSR1 次数。
    int di = -1;  // --stop-di 的输入口，-1 = 不用。
    bool di_last = false;  // 上次读到的电平：低变高才算一次；开始时就是高也算。

    std::string Poll(const rdk::Robot& robot)  // 这次查到的来源；没有返回空串。
    {
        std::string source;
        if (keyboard) {
            pollfd pfd = {STDIN_FILENO, POLLIN, 0};
            if (poll(&pfd, 1, 0) > 0) {
                char buf[256];
                const ssize_t n = (pfd.revents & POLLNVAL) ? 0 : read(STDIN_FILENO, buf, sizeof(buf));
                if (n > 0) {
                    if (std::find(buf, buf + n, '\n') != buf + n) {
                        source = "终端 Enter";
                    }
                } else if (n == 0 || (errno != EINTR && errno != EAGAIN)) {
                    keyboard = false;
                    std::cout << "stdin 已关闭，不再从终端读 Stop Sign（SIGUSR1、--stop-di 照常）" << std::endl;
                }
            }
        }
        if (const uint64_t count = g_sigusr1_count; count != usr1_seen) {
            usr1_seen = count;
            source = "SIGUSR1";
        }
        if (di >= 0) {
            const bool level = robot.digital_inputs()[static_cast<size_t>(di)];
            if (level && !di_last) {
                source = "DI " + std::to_string(di);
            }
            di_last = level;
        }
        return source;
    }
};

// 主线程发出第一个 Stop Sign 后等实时任务锁存，最多 100 ms：锁存后马上切运动控制，不用再等一个 kMonitorPeriod。
void WaitArm1Latched()
{
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(100);
    while (!g_stop
           && g_arm1_state.load(std::memory_order_acquire) == static_cast<int>(soft_stop::Arm1State::TRANSPARENT)
           && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

// 软停止时 ARM_1 的刚度和控制方式。都是阻塞调用，在主线程里调，不在实时回调里：实时任务锁存 SOFT_STOP 后先设刚度、
// 再把 X Y Z 切回运动控制（透明时它们是力控，位姿指令不起作用）；--stiffness-ramp 时之后每 kStiffnessRampPeriod
// 按 K_x(t) 再设一次，直到 t ≥ T 设成停止刚度。ARM_2 的刚度、力控轴不碰。
struct Arm1StopStiffness
{
    soft_stop::Vec6 K0 = {};  // 透明时的实际刚度：力控的 X Y Z 位置刚度不起作用，按 0。
    soft_stop::Vec6 K_stop = {};  // 停止后的刚度。
    bool ramp = false;
    std::array<double, kCartDoF / 2> max_linear_vel = {};  // SetForceControlAxis 原样传回去。
    bool motion_ctrl = false;  // 已切回运动控制。
    bool done = false;  // 不用再设了。
    double next_time = 0.0;  // 渐增时下次设刚度的时刻 [s]
    int impedance_calls = 0;
    double impedance_max_ms = 0.0;
    double axis_ms = 0.0;  // SetForceControlAxis 的用时。

    double SetImpedance(rdk::Robot& robot, const soft_stop::Vec6& K)  // 设 ARM_1 刚度（阻尼比用默认 0.7），返回用时 [ms]。
    {
        const auto start = std::chrono::steady_clock::now();
        robot.SetCartesianImpedance(rdk::JointGroup::ARM_1, K);
        const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
        for (size_t i = 0; i < kCartDoF; ++i) {
            g_arm1_set_K[i] = K[i];
        }
        g_impedance_latency_ms = ms;
        ++impedance_calls;
        impedance_max_ms = std::max(impedance_max_ms, ms);
        return ms;
    }

    void Update(rdk::Robot& robot, const soft_stop::StopPlan& plan)  // 锁存之后每 kMonitorPeriod 调一次。
    {
        if (done) {
            return;
        }
        const double now = SteadySeconds();
        const double t = now - plan.t0;
        if (!motion_ctrl) {
            const auto K = soft_stop::StopStiffness(ramp, K0, K_stop, t, plan.T);
            const double impedance_ms = SetImpedance(robot, K);  // 先设刚度：X Y Z 还是力控不受影响，Rx Ry Rz 从这里开始跟踪。
            const auto start = std::chrono::steady_clock::now();
            robot.SetForceControlAxis(rdk::JointGroup::ARM_1, {false, false, false, false, false, false}, max_linear_vel);
            axis_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
            motion_ctrl = true;
            g_arm1_motion_ctrl = true;
            done = !ramp;
            next_time = now + std::chrono::duration<double>(kStiffnessRampPeriod).count();
            std::cout << "ARM_1 已切回运动控制（锁存后 " << Num((SteadySeconds() - plan.t0) * 1000.0) << " ms）：刚度 ["
                      << rdk::utility::Arr2Str(K, 1) << "]，SetCartesianImpedance " << Num(impedance_ms)
                      << " ms，SetForceControlAxis " << Num(axis_ms) << " ms" << std::endl;
            return;
        }
        if (now < next_time) {
            return;
        }
        SetImpedance(robot, soft_stop::StiffnessAt(K0, K_stop, t, plan.T));
        next_time = now + std::chrono::duration<double>(kStiffnessRampPeriod).count();
        if (t >= plan.T) {
            done = true;
            std::cout << "刚度渐增结束：[" << rdk::utility::Arr2Str(K_stop, 1) << "]，共 " << impedance_calls
                      << " 次 SetCartesianImpedance，最长 " << Num(impedance_max_ms) << " ms" << std::endl;
        }
    }
};

void PrintSoftStopStart(const soft_stop::StopPlan& plan, const std::string& source)  // 锁存后打印一次。
{
    std::cout << "SOFT_STOP START（Stop Sign 来自" << source << "）：ARM_1 |v0| = " << Num(soft_stop::Norm3(plan.v0))
              << " m/s、|ω0| = " << Num(soft_stop::Norm3(plan.w0)) << " rad/s，T = " << Num(plan.T) << " s，计划再走 "
              << Num(plan.distance * 1000.0) << " mm、转 " << Num(rdk::utility::Rad2Deg(plan.angle))
              << "°；ARM_2 照旧 hold。" << std::endl;
}

// 进 HOLD 后打印一次。plan、hold_time 在离开 TRANSPARENT、进 HOLD 之后实时任务不再改。
void PrintHold(const rdk::Robot& robot, const soft_stop::Arm1StopMachine& machine)
{
    const auto& plan = machine.plan();
    const auto pose = robot.states().at(rdk::JointGroup::ARM_1).tcp_pose;
    const double moved = soft_stop::Norm3(pose[0] - plan.p0[0], pose[1] - plan.p0[1], pose[2] - plan.p0[2]);
    std::cout << "ARM_1 HOLD：锁存后 " << Num(machine.hold_time() - plan.t0) << " s 停稳（T = " << Num(plan.T)
              << " s），实际走了 " << Num(moved * 1000.0) << " mm（计划 " << Num(plan.distance * 1000.0)
              << " mm），离终点 " << Num(soft_stop::Distance(pose, plan.stop_pose) * 1000.0)
              << " mm。两条臂都在 hold；要恢复透明，Ctrl+C 结束后重新运行。" << std::endl;
}

// 把实时任务记下的行写进文件：在主线程，不在实时任务里。
void DrainLog(soft_stop::SpscRing<soft_stop::LogRow>& queue, std::ofstream& log, uint64_t& rows)
{
    soft_stop::LogRow row;
    while (queue.TryPop(row)) {
        soft_stop::WriteLogRow(log, row);
        ++rows;
    }
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
        std::error_code exists_error;  // 查不了（例如没有权限）就当不存在，下面 open 失败时照样退出。
        if (std::filesystem::exists(params.log_path, exists_error)) {  // 不覆盖以前的记录。
            std::cerr << "日志文件 " << params.log_path << " 已经存在，为了不覆盖以前的记录，程序不运行。\n"
                         "换个文件名，例如带日期时间：--log logs/soft_stop_$(date +%Y%m%d_%H%M%S).csv\n";
            return 2;
        }
        log.open(params.log_path);
        if (!log) {
            std::cerr << "打不开日志文件 " << params.log_path << "（目录要先建好）\n";
            return 2;
        }
        log.precision(9);
    }

    // 声明顺序决定析构顺序：调度器最先析构，实时任务不会用到已经销毁的 robot、targets 和日志队列。
    std::unique_ptr<rdk::Robot> robot;
    RtTargets targets;
    Arm2Record arm2_record;
    std::unique_ptr<soft_stop::SpscRing<soft_stop::LogRow>> log_queue;
    std::unique_ptr<rdk::Scheduler> scheduler;
    bool need_stop = false, started = false;
    Arm1StopStiffness stiffness;
    uint64_t log_rows = 0;  // 写进日志文件的行数。
    try {
        scheduler = std::make_unique<rdk::Scheduler>();  // 先建调度器：系统问题在连机器人之前就暴露。
        robot = std::make_unique<rdk::Robot>(params.robot_sn);
        for (int sig : {SIGINT, SIGTERM, SIGHUP}) {  // 在 RDK 初始化之后装，保证由这里接管信号。
            std::signal(sig, OnSignal);
        }
        std::signal(SIGUSR1, OnStopSignal);  // 普通 Stop Sign。SIGUSR1 的默认动作是结束进程，所以和上面一起装。
        Prepare(*robot, params);
        const auto info = robot->info();
        std::cout << "机器人 " << info.serial_num << "，控制器软件版本 " << info.software_ver << std::endl;
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
        // ARM_1 软停止的刚度（ARM_2 不碰）。K0 = 透明时的实际刚度：力控的 X Y Z 位置刚度不起作用，按 0。
        // K_stop = 停止后的刚度：能拖的轴 K_x_nom × stop_stiffness_scale，不低于 K0；锁住的 X Y Z 保持 K_x_nom。
        stiffness.K0 = arm1_K;
        stiffness.K_stop = arm1_K;
        for (size_t i = 0; i < kCartDoF; ++i) {
            const bool force_axis = i < 3 && params.force_axis[i];
            const bool locked = i < 3 && !params.force_axis[i];
            stiffness.K0[i] = force_axis ? 0.0 : arm1_K[i];
            stiffness.K_stop[i]
                = locked ? arm1_K_nom[i] : std::max(stiffness.K0[i], arm1_K_nom[i] * params.stop_stiffness_scale);
        }
        stiffness.ramp = params.stiffness_ramp;
        stiffness.max_linear_vel = params.max_linear_vel;

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
        const auto arm1_pose = states.at(rdk::JointGroup::ARM_1).tcp_pose;  // ARM_1 位置控制轴（kForceAxis 为 0）锁定的位姿。
        targets.arm2_pose = states.at(rdk::JointGroup::ARM_2).tcp_pose;  // ARM_2 hold 的末端位姿：只在这里读一次。
        targets.arm2_q = states.at(rdk::JointGroup::ARM_2).q;
        // ARM_1：力控轴目标力 0，位置控制轴锁在 arm1_pose。ARM_2：只给位姿，wrench、twist、acc 都是 0，就是纯位置控制。
        targets.commands = {
            {rdk::JointGroup::ARM_1, rdk::RtCartesianCmd(arm1_pose, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0})},
            {rdk::JointGroup::ARM_2, rdk::RtCartesianCmd(targets.arm2_pose)},
        };
        soft_stop::CartesianCmd transparent_cmd;  // 和上面 ARM_1 的透明指令一样。
        transparent_cmd.pose_d = arm1_pose;
        targets.arm1.emplace(MakeStopConfig(params), transparent_cmd);
        targets.K0 = stiffness.K0;
        targets.K_stop = stiffness.K_stop;
        targets.stiffness_ramp = params.stiffness_ramp;
        for (size_t i = 0; i < kCartDoF; ++i) {
            g_arm1_set_K[i] = stiffness.K0[i];
        }
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
        std::cout << "ARM_1 软停止：减速度上限 " << Num(kStopMaxLinearDecel) << " m/s²、" << Num(kStopMaxAngularDecel)
                  << " rad/s²，T ≥ " << Num(kStopMinTime) << " s，速度大时 T、停止距离跟着变大（不设上限）；锁存后刚度 ["
                  << rdk::utility::Arr2Str(stiffness.K_stop, 1)
                  << "]（透明时实际 [" << rdk::utility::Arr2Str(stiffness.K0, 1) << "]），刚度渐增"
                  << (params.stiffness_ramp ? "开（第二阶段实验）" : "关") << std::endl;
        if (log.is_open()) {
            log_queue = std::make_unique<soft_stop::SpscRing<soft_stop::LogRow>>(kLogQueueRows);
            targets.log = log_queue.get();
            soft_stop::WriteLogHeader(log);
        }
        StopSignInput stop_input;
        stop_input.di = params.stop_di;
        stop_input.usr1_seen = g_sigusr1_count;  // ③ 之前收到的 SIGUSR1 不算。
        tcflush(STDIN_FILENO, TCIFLUSH);  // ①② 期间在终端按的键不算；stdin 不是终端时什么也不做。
        scheduler->AddTask(
            [&] { PeriodicTask(*robot, targets, arm2_record); }, "HP periodic", 1, scheduler->max_priority());
        targets.start_time = SteadySeconds();
        scheduler->Start();
        started = true;
        std::cout << "③ 实时模式 RT_CARTESIAN_MOTION_FORCE 已开始：ARM_1 可以手拖；ARM_2 hold 不动。按 Ctrl+C 结束。"
                  << std::endl;
        std::cout << "发 Stop Sign：终端按 Enter，或另开终端 sudo kill -USR1 " << getpid()
                  << (params.stop_di >= 0 ? "，或 DI " + std::to_string(params.stop_di) + " 变高" : std::string())
                  << "。ARM_1 减速到 0 后 HOLD，ARM_2 照旧 hold；不是急停，急停用 E-stop。" << std::endl;
        auto printed_state = soft_stop::Arm1State::TRANSPARENT;
        std::string stop_source;
        while (!g_stop) {
            if (const auto reason = NotRunningReason(*robot, rdk::Mode::RT_CARTESIAN_MOTION_FORCE); !reason.empty()) {
                RequestStop(reason);
            }
            if (const auto source = stop_input.Poll(*robot); !source.empty()) {
                if (++g_stop_sign_count == 1) {
                    stop_source = source;
                    WaitArm1Latched();
                } else {
                    std::cout << "又收到 Stop Sign（" << source << "）：ARM_1 软停止已经锁存，不重新开始" << std::endl;
                }
            }
            const auto state = static_cast<soft_stop::Arm1State>(g_arm1_state.load(std::memory_order_acquire));
            if (state != soft_stop::Arm1State::TRANSPARENT && !g_stop) {
                const auto& machine = *targets.arm1;  // 只读 plan、hold_time：离开 TRANSPARENT 之后实时任务不再改它们。
                if (printed_state == soft_stop::Arm1State::TRANSPARENT) {
                    PrintSoftStopStart(machine.plan(), stop_source);
                }
                stiffness.Update(*robot, machine.plan());
                if (state == soft_stop::Arm1State::HOLD && printed_state != soft_stop::Arm1State::HOLD) {
                    PrintHold(*robot, machine);
                }
                printed_state = state;
            }
            if (log_queue) {
                DrainLog(*log_queue, log, log_rows);
            }
            std::this_thread::sleep_for(kMonitorPeriod);
        }
    } catch (const std::exception& e) {
        RequestStop(e.what());
    }

    g_stop = true;  // 出错路径上也让实时任务停止发指令。
    for (int sig : {SIGINT, SIGTERM, SIGHUP, SIGUSR1}) {  // 停机过程中不再被信号打断。
        std::signal(sig, SIG_IGN);
    }
    bool record_ready = false;  // 调度器正常停下后，arm2_record、targets 才不会再被实时任务改。
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
        if (targets.arm1) {
            std::cout << "ARM_1 软停止：收到 Stop Sign " << g_stop_sign_count.load() << " 次，最后状态 "
                      << soft_stop::StateName(targets.arm1->state());
            if (stiffness.impedance_calls > 0) {
                std::cout << "；SetCartesianImpedance(ARM_1) " << stiffness.impedance_calls << " 次，最长 "
                          << Num(stiffness.impedance_max_ms) << " ms，SetForceControlAxis(ARM_1) "
                          << Num(stiffness.axis_ms) << " ms";
            }
            std::cout << "。实时周期间隔最长 " << Num(targets.max_cycle_ms) << " ms，超过 " << Num(kLateCycleMs)
                      << " ms 的 " << targets.late_cycles << " 次" << std::endl;
        }
    }
    if (log_queue) {
        DrainLog(*log_queue, log, log_rows);
        log.close();
        std::cout << "日志已写入 " << params.log_path << "（" << log_rows << " 行，队列满丢了 " << g_log_dropped.load()
                  << " 行）" << std::endl;
    }

    std::lock_guard<std::mutex> lock(g_reason_mutex);
    if (!g_reason.empty()) {
        std::cerr << "已停止：" << g_reason << std::endl;
        return 1;
    }
    return 0;
}
