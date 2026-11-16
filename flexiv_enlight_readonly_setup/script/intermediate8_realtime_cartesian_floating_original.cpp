// RT 笛卡尔浮动：ARM_1 末端 X Y Z Rx Ry Rz 每个轴由 kLockX … kLockRz 决定能拖还是锁定（默认 X Y Z 能拖、Rx Ry Rz 锁定），
// 能拖的轴尽可能透明；ARM_2 全程不动。
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
//   ARM_1：目标力 0，不自己解 IK。先用名义刚度 K_x_nom、目标位姿锁在进入 ③ 时的末端位姿，成功发满 kWarmupCycles 次，
//          确认实时指令一直在发；然后能拖的轴（kLockX … kLockRz 为 false）目标每 1 ms 跟随实测值，再把这些轴的刚度
//          降到 K_x_nom × kStiffnessScale；--passive、--active 再给能拖的 X Y Z 开被动、主动零力控制（Rx Ry Rz 不做力控，
//          照样按刚度浮动）。锁定的轴目标不跟随、刚度保持 K_x_nom。
//          降刚度、开力控都是实时模式里的参数设置，不切换控制模式，两条臂的指令照常发。
//          跟随实测位姿是和所附方案不同的地方：方案的目标位姿一直锁在起点，刚度 0.10 时 ARM_1 像被弹簧拴住，推不动、
//          松手弹回（11-03 实测）。跟随之后弹簧力约为 0，只剩阻尼：哪里都能拖，松手停在原地，没人推时是匀速漂、不会越漂越快。
//   ARM_2：6 轴纯位置控制、名义刚度，目标位姿一直是进入 ③ 时的末端位姿（官方 intermediate5 的 --hold）。
//          实时模式下没有 Hold 原语可用，ARM_2 的 hold 指令就是每 1 ms 发一次的这个位姿。
// 7 轴臂的肘部（零空间）不归末端刚度管，由零空间姿态控制拉回进入 ③ 时的关节姿态，所以肘部不像关节版那样完全自由；
// --nullspace-follow 时参考姿态跟随实测关节位置，基本不再拉回。
//
// 编译（项目根目录）：cmake -S script -B .build/script && cmake --build .build/script
// 运行（实时调度器需要 root）：sudo .build/script/intermediate8_realtime_cartesian_floating <序列号> [选项]；--help 看选项。

#include <flexiv/rdk/robot.hpp>
#include <flexiv/rdk/scheduler.hpp>
#include <flexiv/rdk/utility.hpp>

#include <Eigen/Geometry>  // FollowPose 的四元数运算；RDK 本身依赖 Eigen 3.4。
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
#include <filesystem>
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

// ================================ 透明度参数：在这里调 ================================
// 都是 RDK 的官方参数，也可以用命令行参数临时覆盖，不用重新编译。
// RDK 的笛卡尔模式没有逐关节（J1–J7）阻尼：SetJointImpedance 的 Z_q 只能在关节阻抗模式下用。这里的阻尼都是末端
// X Y Z Rx Ry Rz 每个轴各一个，由下面两个参数决定。
// kStiffnessScale：ARM_1 能拖的轴（kLockX … kLockRz 为 false）的笛卡尔刚度 = K_x_nom × kStiffnessScale，按 X Y Z Rx Ry Rz
//   每个轴一个比例（SetCartesianImpedance 的 K_x，K_x_nom 启动时打印）；锁定的轴保持 K_x_nom，不看这里的值。
//   范围 0–1；0 = 零刚度浮动，最透明（文档："Setting motion stiffness of a motion-controlled Cartesian axis to 0 will
//   make this axis free-floating"）。方案建议 0.10 → 0.05 → 0.02 → 0.00 一档一档降。
//   命令行 --scale=S（6 个轴都用 S）或 --scale=S1,S2,S3,S4,S5,S6。
//   浮动时目标位置跟随实测位置，刚度几乎不产生弹簧力，实际起作用的是阻尼：按阻尼比的定义，阻尼 = 2 × Z_x × √(刚度 × 惯量)，
//   所以比例越小阻尼越小、越透明（阻尼 ∝ √比例）；0 时阻尼也是 0，没人推会越漂越快。
//   程序不设速度限位，只能靠 E-stop 和控制器自带的保护停。
constexpr std::array<double, rdk::kCartDoF> kStiffnessScale = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
// kDampingRatio：ARM_1 能拖的轴浮动时的阻尼比（SetCartesianImpedance 的 Z_x），按 X Y Z Rx Ry Rz 排列；锁定的轴和预热时
//   都用 RDK 默认 0.7，不看这里的值。范围 0.3–0.8，RDK 默认 0.7；文档警告改它可能影响性能和稳定性。
//   同一个刚度下只能把阻尼调到 0.7 时的 0.43–1.14 倍；刚度为 0 的轴阻尼一直是 0，改它不起作用，大范围调用 kStiffnessScale。
//   --passive、--active 时能拖的 X Y Z 是力控，这几个轴不起作用。命令行 --zeta=Z（6 个轴都用 Z）或 --zeta=Z1,Z2,Z3,Z4,Z5,Z6。
constexpr std::array<double, rdk::kCartDoF> kDampingRatio = {0.3,0.3,0.3,0.3,0.3,0.3};
constexpr double kNominalDampingRatio = 0.1;  // RDK 的 SetCartesianImpedance 默认 Z_x：锁定的轴和预热时用，不是参数。
// kNullSpaceRefTracking：SetNullSpaceObjectives 的 ref_positions_tracking，肘部（零空间）拉回进入 ③ 时关节姿态的权重。
//   范围 0.1–1.0，RDK 默认 0.5；越小肘部越自由。0.1 是最小值，不等于关掉零空间控制。另外两个权重用 RDK 默认 0。
//   命令行 --nullspace=W。
constexpr double kNullSpaceRefTracking = 0.1;
// kPassiveForceControl：true = 被动零力控制（方案的 passive zero-wrench）：SetPassiveForceControl 打开，能拖的 X Y Z 做力控、
//   目标力 0，只前馈 0 力，不加主动力控的阻尼；这几个轴上 kStiffnessScale、kDampingRatio 不起作用，
//   SetForceControlAxis 的线速度保护也不起作用。锁定的轴照样位置控制。
//   Rx Ry Rz 不做力控：力矩控制要 6 维力传感器，10-30 实测这台报错 "moment control is only available with 6DoF F/T sensor"。
//   能拖的 Rx Ry Rz 和不开 --passive 时一样，按 kStiffnessScale、kDampingRatio 浮动（0 = 零刚度自由转）。
//   不能和 kActiveForceControl 同时为 true。命令行 --passive。
constexpr bool kPassiveForceControl = true;
// kActiveForceControl：true = 主动零力控制：能拖的 X Y Z 轴做力控、目标力 0，闭环用关节力矩估算的末端力把手上的力压到 0，
//   原以为能抵掉一部分关节摩擦和惯量，11-06 实测反而重得多：平移阻尼 141–202 N·s/m，不开时 7–10 N·s/m
//   （compare_transparency.py，logs/i8_active*.csv 对 i8_ns010、i8_nsfollow），就是文档说的主动力控多出来的阻尼。
//   Rx Ry Rz 照样位置控制、刚度
//   K_x_nom × kStiffnessScale，不用 6 维力传感器。X Y Z 上 kStiffnessScale、kDampingRatio 不起作用。
//   代价：文档说主动力控会多一点笛卡尔阻尼；末端力估算有偏差时没人推也会自己走，最快到 kActiveMaxLinearVel。
//   不能和 kPassiveForceControl 同时为 true。命令行 --active。
constexpr bool kActiveForceControl = false;
// kActiveMaxLinearVel：SetForceControlAxis 的 max_linear_vel，主动力控时 X Y Z 末端线速度上限 [m/s]。范围 0.005–2.0，
//   RDK 默认 1.0。这是控制器自己的限速，不停机；被动力控下不起作用。
constexpr std::array<double, rdk::kCartDoF / 2> kActiveMaxLinearVel = {1.0, 1.0, 1.0};
// kNullSpaceFollow：true = 肘部参考姿态跟随实测关节位置：浮动后主线程每 kMonitorPeriod 把 SetNullSpacePosture 设成
//   当前关节位置，零空间几乎不再把肘部拉回进入 ③ 时的姿态，kNullSpaceRefTracking 的拉力约为 0。
//   肘部自运动（J1 J2 J3 一起转、末端不动的那种）变轻；代价是没人推时肘部可能自己漂。命令行 --nullspace-follow。
constexpr bool kNullSpaceFollow = false;
// kLockX … kLockRz：ARM_1 末端每个轴锁不锁，没有命令行参数，改了要重新编译。
//   true = 锁定：这个轴的目标一直是进入 ③ 时的值（不跟随实测），刚度保持名义刚度 K_x_nom（RDK 允许的最大值），
//          和 ARM_2 hold 一样硬；--passive 时也不做力控。
//   false = 能拖：目标跟随实测值，刚度 = K_x_nom × kStiffnessScale，阻尼比 = kDampingRatio；--passive 时 X Y Z 做零力控制。
//   都是世界坐标系的轴：RDK 的笛卡尔运动控制一直用世界坐标系，和 SetCartesianImpedance 的 K_x 同一个顺序。
//   转动轴锁定的意思是：从进入 ③ 时的姿态转到当前姿态的旋转向量（世界坐标系）在这个轴上的分量拉回 0，见 FollowPose。
//   默认和原来的 kLockRotation = true 一样：X Y Z 能拖，Rx Ry Rz 锁定。
constexpr bool kLockX = false;
constexpr bool kLockY = false;
constexpr bool kLockZ = false;
constexpr bool kLockRx = false;
constexpr bool kLockRy = false;
constexpr bool kLockRz = false;
// 目标位姿（预热时是进入 ③ 时的位姿；浮动时能拖的轴跟随实测值，锁定的轴不跟随）、目标力固定为 0，不是参数。
// ====================================================================================

// 程序内部用，由上面 6 个参数得出，不用改：按 X Y Z Rx Ry Rz 排列。
constexpr std::array<bool, rdk::kCartDoF> kLockAxis = {kLockX, kLockY, kLockZ, kLockRx, kLockRy, kLockRz};
const std::array<std::string, rdk::kCartDoF> kAxisNames = {"X", "Y", "Z", "Rx", "Ry", "Rz"};
constexpr bool kAnyRotationFree = !kLockRx || !kLockRy || !kLockRz;  // Rx Ry Rz 里至少一个能拖。
constexpr bool kAllRotationFree = !kLockRx && !kLockRy && !kLockRz;  // Rx Ry Rz 三个都能拖。

// ===== ③ ARM_2 hold：发给机器人，只影响 ARM_2，不影响 ARM_1 手感 =====
// kArm2NullSpaceRefTracking：ARM_2 的 SetNullSpaceObjectives ref_positions_tracking，肘部拉回 hold 姿态的权重。
//   范围 0.1–1.0，RDK 默认 0.5；取上限 1.0，拉得最紧。11-03 实测默认 0.5 时 ARM_2 末端不动，但 J1、J3 反向转
//   （J2 ≈ -0.09，J1 J3 近乎共轴，是零空间自运动），约 4 秒超过当时 0.01 rad 的停机检查。另外两个权重用 RDK 默认 0。
constexpr double kArm2NullSpaceRefTracking = 1.0;

// ===== ① 回正、② 受力回 0：原语参数，同 arm1_transparent_arm2_hold_rt.cpp，照抄 Flexiv Primitives 3E.1 文档，不影响手感 =====
constexpr int kHomeJntVelScale = 20;  // Home 的 jntVelScale：关节速度百分比，100 为全速。范围 1–100，文档默认 20。
// Home 的 target 不传，用文档默认的 home 姿态 0 -40 0 90 0 -50 0 deg。
// ZeroFTSensor（Basic Force Control 页）：清零时机器人要静止，不能被碰、不能接触环境。程序等 ARM_1 回正停稳后才清零。
constexpr double kZeroDataCollectTime = 0.2;  // dataCollectTime：采样多久来定零点 [s]。范围 0.1–1.0，文档默认 0.2。
constexpr int kZeroEnableStaticCheck = 0;  // enableStaticCheck：清零前检查机器人是否静止，1 检查。文档默认 0。
constexpr int kZeroCalibExtraPayload = 0;  // calibExtraPayload：标定时是否把额外负载算进工具负载，1 算。文档默认 0。

// ========== 停机条件和流程：不发给机器人、不影响手感，只决定什么时候整机停 ==========
// 11-03 按要求不设速度、位置限位类的停机检查：ARM_1 末端速度、离起点的位移、关节离极限的余量、关节速度，
// ARM_2 偏离 hold 位置，都不检查（ARM_2 的偏离仍然记录，结束时打印）。
// 只在机器人自己不能继续时停：断线、E-stop、故障、未就绪、控制模式被切走、实时指令迟到过多、实时任务心跳丢失。
// 控制器自带的保护（关节硬极限、安全设置里的限速、碰撞检测等）RDK 关不掉，照常有效。
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
    std::array<double, rdk::kCartDoF> stiffness_scale = kStiffnessScale;
    std::array<double, rdk::kCartDoF> damping_ratio = kDampingRatio;
    double null_space_tracking = kNullSpaceRefTracking;
    bool passive = kPassiveForceControl;
    bool active = kActiveForceControl;
    bool null_space_follow = kNullSpaceFollow;
    std::string log_path;  // 空 = 不记录。
    bool home = true;
    bool clear_fault = false;
};

struct Arm2Record  // ③ 期间 ARM_2 最大偏离 hold 位置 [rad] 和最大速度 [rad/s]：实时任务写，调度器停下后主线程读。
{
    std::array<double, kDoF> max_error = {};
    std::array<double, kDoF> max_vel = {};
};

struct RtTargets  // ③ 的指令和记录用的数据：进入 ③ 时定好，调度器启动后只有实时任务读写，主线程不再碰。
{
    // 每 1 ms 发出的两条臂指令。g_arm1_follow 之后实时任务每次用 FollowPose 更新 ARM_1 的目标位姿，其余不变。
    std::map<rdk::JointGroup, rdk::RtCartesianCmd> commands;
    std::array<double, rdk::kPoseSize> arm1_start_pose = {};  // 进入 ③ 时 ARM_1 的末端位姿：锁定的轴一直用它。
    std::vector<double> arm2_q;  // ARM_2 hold 时的关节位置 [rad]，只用来记录 ARM_2 动了多少，不停机。
};

std::atomic<bool> g_stop = {false};  // 置 true 后实时任务不再发指令，主线程收尾。信号处理函数只做这一件事。
std::mutex g_reason_mutex;  // 保护 g_reason：实时线程和主线程都可能写。
std::string g_reason;  // 第一个停机原因；空 = Ctrl+C 正常结束。
std::atomic<uint64_t> g_sent_count = {0};  // ③ 实时任务成功发出指令的次数。
std::atomic<int64_t> g_last_sent_ns = {0};  // ③ 实时任务最近一次成功发出指令的时刻 [ns]，见 NowNs。
std::atomic<bool> g_arm1_follow = {false};  // ③ 降刚度前主线程置 true：之后 ARM_1 的目标位置每 1 ms 跟随实测位置。

void PrintHelp()
{
    std::cout << "用法：sudo .build/script/intermediate8_realtime_cartesian_floating <序列号> [选项]\n"
                 "  <序列号>          完整序列号，不要有空格，例如 Enlight-LL-123456\n"
                 "  --scale=S         覆盖 kStiffnessScale：ARM_1 能拖的轴（kLockX … kLockRz 为 false）的刚度占名义刚度的\n"
                 "                    比例 [0, 1]；目标跟随实测值，实际决定阻尼，越小越透明；0 = 无阻尼浮动。\n"
                 "                    1 个数 = 6 个轴都用它；逗号分开的 6 个数 = X,Y,Z,Rx,Ry,Rz 各一个，例如 --scale=0.05,0.05,0.02,0,0,0\n"
                 "  --zeta=Z          覆盖 kDampingRatio：ARM_1 能拖的轴的阻尼比 [0.3, 0.8]，RDK 默认 0.7；刚度为 0 的轴不起作用。\n"
                 "                    1 个数或逗号分开的 6 个数，同 --scale；--passive、--active 时对 X Y Z 不起作用\n"
                 "  --nullspace=W     覆盖 kNullSpaceRefTracking：肘部拉回参考姿态的权重 [0.1, 1]\n"
                 "  --passive         覆盖 kPassiveForceControl：能拖的 X Y Z 做被动零力控制；Rx Ry Rz 不做力控，\n"
                 "                    按 --scale、--zeta 浮动（不用 6 维力传感器）\n"
                 "  --active          覆盖 kActiveForceControl：能拖的 X Y Z 轴做主动零力控制；11-06 实测比不开重得多（阻尼约 20 倍）；\n"
                 "                    没人推时可能自己走，最快到 kActiveMaxLinearVel；不能和 --passive 同时用\n"
                 "  --nullspace-follow\n"
                 "                    覆盖 kNullSpaceFollow：肘部参考姿态每 10 ms 跟随实测关节位置，肘部不再被拉回；\n"
                 "                    没人推时肘部可能自己漂\n"
                 "  --log=FILE.csv    ③ 浮动期间按 100 Hz 记录 ARM_1 的末端位姿、速度、外力和关节位置、速度；\n"
                 "                    FILE.csv 已经存在时不运行，不覆盖旧记录，文件名建议带日期时间\n"
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

std::string AxisNames(bool locked)  // kLockAxis 里等于 locked 的轴，例如 "X Y Z"；一个都没有时是 "无"。
{
    std::string text;
    for (size_t i = 0; i < rdk::kCartDoF; ++i) {
        if (kLockAxis[i] == locked) {
            text += (text.empty() ? "" : " ") + kAxisNames[i];
        }
    }
    return text.empty() ? "无" : text;
}

bool ParseNumber(const char* text, double& value)  // 整个字符串是一个有限的数才算成功。
{
    char* end = nullptr;
    value = std::strtod(text, &end);
    return end != text && *end == '\0' && std::isfinite(value);
}

// text 是 1 个数（6 个轴都用它）或逗号分开的 6 个数（X Y Z Rx Ry Rz 各一个）才算成功。
bool ParseAxisValues(const std::string& text, std::array<double, rdk::kCartDoF>& values)
{
    std::vector<double> parsed;
    std::istringstream iss(text);
    std::string item;
    while (std::getline(iss, item, ',')) {
        double value = 0.0;
        if (!ParseNumber(item.c_str(), value)) {
            return false;
        }
        parsed.push_back(value);
    }
    if (text.ends_with(",") || (parsed.size() != 1 && parsed.size() != values.size())) {
        return false;
    }
    for (size_t i = 0; i < values.size(); ++i) {
        values[i] = parsed.size() == 1 ? parsed[0] : parsed[i];
    }
    return true;
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
            if (!ParseAxisValues(value, params.stiffness_scale)) {
                return "--scale= 后面要跟 1 个数或逗号分开的 6 个数";
            }
        } else if (OptionValue(arg, "--zeta", value)) {
            if (!ParseAxisValues(value, params.damping_ratio)) {
                return "--zeta= 后面要跟 1 个数或逗号分开的 6 个数";
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
        } else if (arg == "--active") {
            params.active = true;
        } else if (arg == "--nullspace-follow") {
            params.null_space_follow = true;
        } else if (arg == "--no-home") {
            params.home = false;
        } else if (arg == "--clear-fault") {
            params.clear_fault = true;
        } else {
            return "不认识的参数 " + arg;
        }
    }
    // 下发前按 RDK 文档范围检查；上方常量也一样要过这一关。刚度上限 K_x_nom 由比例 ≤ 1 保证。
    for (size_t i = 0; i < rdk::kCartDoF; ++i) {
        if (!(params.stiffness_scale[i] >= 0.0 && params.stiffness_scale[i] <= 1.0)) {
            return "stiffness_scale 的 " + kAxisNames[i] + " = " + Num(params.stiffness_scale[i]) + " 超出范围 [0, 1]";
        }
        if (!(params.damping_ratio[i] >= 0.3 && params.damping_ratio[i] <= 0.8)) {
            return "damping_ratio 的 " + kAxisNames[i] + " = " + Num(params.damping_ratio[i]) + " 超出范围 [0.3, 0.8]";
        }
    }
    if (!(params.null_space_tracking >= 0.1 && params.null_space_tracking <= 1.0)) {
        return "null_space_tracking = " + Num(params.null_space_tracking) + " 超出范围 [0.1, 1]";
    }
    for (double v : kActiveMaxLinearVel) {
        if (!(v >= 0.005 && v <= 2.0)) {
            return "kActiveMaxLinearVel 里的 " + Num(v) + " 超出范围 [0.005, 2]";
        }
    }
    if (params.active && params.passive) {
        return "--active 和 --passive 只能选一个（kActiveForceControl、kPassiveForceControl 不能都是 true）";
    }
    if ((params.active || params.passive) && kLockX && kLockY && kLockZ) {
        return std::string("X Y Z 都锁定，") + (params.active ? "--active" : "--passive") + " 没有能做力控的轴";
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

std::array<double, kDoF> Error(const std::vector<double>& q, const std::vector<double>& target)  // q - target [rad]。
{
    std::array<double, kDoF> error;
    for (size_t i = 0; i < kDoF; ++i) {
        error[i] = q[i] - target[i];
    }
    return error;
}

int64_t NowNs()  // steady_clock 的当前时刻 [ns]：实时线程记录、主线程比较，用来检查心跳。
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

// 浮动时 ARM_1 的目标位姿：能拖的轴跟随实测值 measured，锁定的轴保持 start（进入 ③ 时的位姿），结果写进 pose_d。
// 位姿都是 RDK 的 [x y z qw qx qy qz]，世界坐标系。每 1 ms 在实时任务里调用，只用固定大小的数据，不分配内存。
void FollowPose(const std::array<double, rdk::kPoseSize>& measured, const std::array<double, rdk::kPoseSize>& start,
    std::array<double, rdk::kPoseSize>& pose_d)
{
    for (size_t i = 0; i < 3; ++i) {  // X Y Z 各管各的。
        pose_d[i] = kLockAxis[i] ? start[i] : measured[i];
    }
    if (!kAnyRotationFree) {  // Rx Ry Rz 都锁定：姿态一直是进入 ③ 时的值（原来 kLockRotation = true 的做法）。
        std::copy_n(start.begin() + 3, 4, pose_d.begin() + 3);
        return;
    }
    if (kAllRotationFree) {  // Rx Ry Rz 都能拖：姿态跟随实测值。
        std::copy_n(measured.begin() + 3, 4, pose_d.begin() + 3);
        return;
    }
    // 只锁一部分转动轴：姿态是一个整体，不能逐个分量拷贝。e = 起始姿态 × 实测姿态⁻¹ 的旋转向量（世界坐标系，从实测姿态
    // 转回起始姿态），把能拖的轴上的分量清零得到 e_lock，目标姿态 = exp(e_lock) × 实测姿态。这样控制器看到的姿态误差
    // 正好是 e_lock：锁定的轴被拉回，能拖的轴上没有弹簧力。
    const Eigen::Quaterniond q_start = Eigen::Quaterniond(start[3], start[4], start[5], start[6]).normalized();
    const Eigen::Quaterniond q_meas = Eigen::Quaterniond(measured[3], measured[4], measured[5], measured[6]).normalized();
    const Eigen::AngleAxisd error(q_start * q_meas.conjugate());
    Eigen::Vector3d e_lock = error.angle() * error.axis();
    for (Eigen::Index i = 0; i < 3; ++i) {
        if (!kLockAxis[static_cast<size_t>(3 + i)]) {
            e_lock[i] = 0.0;
        }
    }
    Eigen::Quaterniond q_d = q_meas;
    if (const double angle = e_lock.norm(); angle > 0.0) {
        q_d = Eigen::Quaterniond(Eigen::AngleAxisd(angle, e_lock / angle)) * q_meas;
    }
    if (q_d.dot(q_meas) < 0.0) {  // q 和 -q 是同一个姿态，取和实测值同号的那个。
        q_d.coeffs() *= -1.0;
    }
    pose_d[3] = q_d.w();
    pose_d[4] = q_d.x();
    pose_d[5] = q_d.y();
    pose_d[6] = q_d.z();
}

// 实时任务：每 1 ms 记录一次 ARM_2 的偏离（不停机），然后把两条臂的末端指令放进同一条 StreamCartesianMotionForce 发出去。
void PeriodicTask(rdk::Robot& robot, RtTargets& targets, Arm2Record& record)
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

        // ARM_1：预热时是进入 ③ 时的位姿；浮动时能拖的轴跟随实测值，锁定的轴保持进入 ③ 时的值（FollowPose）。
        //        目标力 0、目标速度 0，所以能拖的轴只剩阻尼挡着手。
        // ARM_2：进入 ③ 时的位姿，就是 hold。两条臂每次都一起发。
        if (g_arm1_follow) {
            FollowPose(arm1.tcp_pose, targets.arm1_start_pose, targets.commands.at(rdk::JointGroup::ARM_1).pose_d);
        }
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
// done 收到 ARM_1 这条原语的状态，它已经不在运行时收到 nullptr。超时、机器人不能运行、Ctrl+C 都抛出。
void RunArm1Primitive(rdk::Robot& robot, const rdk::PrimitiveArgs& arm1_primitive, double timeout,
    const std::function<bool(const rdk::PrimitiveStates*)>& done)
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
        std::error_code exists_error;  // 查不了（例如没有权限）就当不存在，下面 open 失败时照样退出。
        if (std::filesystem::exists(params.log_path, exists_error)) {  // 不覆盖以前的记录。
            std::cerr << "日志文件 " << params.log_path << " 已经存在，为了不覆盖以前的记录，程序不运行。\n"
                         "换个文件名，例如带日期时间：--log=logs/i8_ns010_$(date +%Y%m%d_%H%M%S).csv\n";
            return 2;
        }
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
    bool need_stop = false, started = false, logging = false, floating = false;
    uint64_t rt_mode_checks = 0;  // 拖动期间主线程查到控制模式是 RT_CARTESIAN_MOTION_FORCE 的次数，结束时打印。
    uint64_t null_space_updates = 0;  // --nullspace-follow：SetNullSpacePosture 调用次数，结束时打印。
    double null_space_max_ms = 0.0;  // --nullspace-follow：SetNullSpacePosture 单次最长用时 [ms]，结束时打印。
    double drag_seconds = 0.0;  // 拖动（ARM_1 已浮动之后）持续的秒数。
    try {
        scheduler = std::make_unique<rdk::Scheduler>();  // 先建调度器：系统问题在连机器人之前就暴露。
        robot = std::make_unique<rdk::Robot>(params.robot_sn);
        for (int sig : {SIGINT, SIGTERM, SIGHUP}) {  // 在 RDK 初始化之后装，保证由这里接管信号。
            std::signal(sig, OnSignal);
        }
        Prepare(*robot, params);
        const auto info = robot->info();
        const auto& arm1_K_nom = info.K_x_nom.at(rdk::JointGroup::ARM_1);
        const auto& arm1_q_min = info.q_min.at(rdk::JointGroup::ARM_1);  // --nullspace-follow 夹参考姿态用。
        const auto& arm1_q_max = info.q_max.at(rdk::JointGroup::ARM_1);
        // ARM_1 浮动时的刚度和阻尼比：能拖的轴 = K_x_nom × stiffness_scale、damping_ratio，锁定的轴保持 K_x_nom、0.7。
        auto arm1_float_K = arm1_K_nom;
        std::array<double, rdk::kCartDoF> arm1_float_Z;
        arm1_float_Z.fill(kNominalDampingRatio);
        std::string zeta_unused;  // 刚度为 0、阻尼比又不是 0.7 的能拖的轴：阻尼一直是 0，改阻尼比不起作用，启动时提示。
        for (size_t i = 0; i < arm1_float_K.size(); ++i) {
            if (!kLockAxis[i]) {
                arm1_float_K[i] *= params.stiffness_scale[i];
                arm1_float_Z[i] = params.damping_ratio[i];
                // --passive、--active 时 X Y Z 力控，刚度、阻尼比本来就不起作用。
                const bool force_axis = (params.passive || params.active) && i < 3;
                if (!force_axis && arm1_float_K[i] == 0.0 && arm1_float_Z[i] != kNominalDampingRatio) {
                    zeta_unused += (zeta_unused.empty() ? "" : " ") + kAxisNames[i];
                }
            }
        }

        need_stop = true;  // 从这里起结束时必须 Stop()。
        if (robot->mode() != rdk::Mode::IDLE) {
            robot->Stop();  // 机器人已静止，Stop() 只是切到 IDLE。
        }
        robot->SetPassiveForceControl(rdk::JointGroup::ARM_1, params.passive);  // 文档：只能在 IDLE 下调用。
        // ①② 用原语，只能在这个非实时模式下运行；这时还不能拖 ARM_1。拖动只在 ③，③ 全程是实时模式。
        robot->SwitchMode(rdk::Mode::NRT_PRIMITIVE_EXECUTION);
        if (params.home) {
            std::cout << "① ARM_1 回正中（Home，关节速度 " << kHomeJntVelScale << "%），ARM_2 Hold。Ctrl+C 可随时停 ……"
                      << std::endl;
            RunArm1Primitive(*robot, rdk::PrimitiveArgs("Home", {{"jntVelScale", kHomeJntVelScale}}), kHomeTimeout,
                [&](const rdk::PrimitiveStates* state) {
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
            kZeroTimeout, [](const rdk::PrimitiveStates* state) {
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
        robot->SetForceControlFrame(rdk::JointGroup::ARM_1, rdk::CoordType::WORLD);  // RDK 默认；只在 --passive、--active 开力控时用到。
        robot->SetNullSpaceObjectives(rdk::JointGroup::ARM_1, 0.0, 0.0, params.null_space_tracking);
        robot->SetNullSpaceObjectives(rdk::JointGroup::ARM_2, 0.0, 0.0, kArm2NullSpaceRefTracking);
        const auto states = robot->states();
        const auto& arm1 = states.at(rdk::JointGroup::ARM_1);
        const auto& arm2 = states.at(rdk::JointGroup::ARM_2);
        robot->SetNullSpacePosture(rdk::JointGroup::ARM_1, arm1.q);  // 肘部参考姿态 = 进入 ③ 时的姿态。
        robot->SetNullSpacePosture(rdk::JointGroup::ARM_2, arm2.q);  // 切模式时已自动设成当前姿态，这里写明，和 hold 位置一致。
        targets.arm2_q = arm2.q;
        targets.arm1_start_pose = arm1.tcp_pose;
        targets.commands = {
            {rdk::JointGroup::ARM_1, rdk::RtCartesianCmd(arm1.tcp_pose, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0})},
            {rdk::JointGroup::ARM_2, rdk::RtCartesianCmd(arm2.tcp_pose)},  // 只给位姿，就是纯位置控制。
        };
        // 能拖的轴浮动时怎么控制，启动时打印。
        std::string float_mode = "刚度比例 [" + rdk::utility::Arr2Str(params.stiffness_scale, 3) + "]，浮动时六轴刚度 ["
                                 + rdk::utility::Arr2Str(arm1_float_K, 1) + "]、阻尼比 ["
                                 + rdk::utility::Arr2Str(arm1_float_Z, 2) + "]";
        if (params.passive) {
            float_mode += "；其中能拖的 X Y Z 做被动零力控制（这几个轴的刚度、阻尼比不起作用），Rx Ry Rz 不做力控";
        } else if (params.active) {
            float_mode += "；其中能拖的 X Y Z 做主动零力控制（这几个轴的刚度、阻尼比不起作用），线速度上限 ["
                          + rdk::utility::Arr2Str(kActiveMaxLinearVel, 3) + "] m/s";
        }
        std::cout << "ARM_1：名义刚度 K_x_nom [" << rdk::utility::Arr2Str(arm1_K_nom, 1) << "] N/m、Nm/rad；能拖的轴 "
                  << AxisNames(false) << "：" << float_mode << "；锁定的轴 " << AxisNames(true)
                  << "：名义刚度、阻尼比 0.7、目标不跟随；肘部拉回权重 " << Num(params.null_space_tracking)
                  << (params.null_space_follow ? "，参考姿态每 " + std::to_string(kMonitorPeriod.count())
                                                     + " ms 跟随实测关节位置"
                                               : std::string("，参考姿态 = 进入 ③ 时的姿态"))
                  << std::endl;
        if (!zeta_unused.empty()) {
            std::cout << "注意：" << zeta_unused << " 刚度为 0，阻尼也是 0，--zeta 对这些轴不起作用；要有阻尼先调大 --scale"
                      << std::endl;
        }
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
            // 先让目标位姿跟随实测位姿，再降刚度：降的那一下不会被拉回起点。
            g_arm1_follow = true;
            // --passive、--active：能拖的 X Y Z 做零力控制、目标力 0。Rx Ry Rz 不做力控（力矩控制要 6 维力传感器，
            // 这台没有，10-30 实测），能拖的 Rx Ry Rz 和不做力控时一样，按下面的刚度、阻尼比浮动；锁定的轴照样位置控制。
            if (params.passive || params.active) {
                robot->SetForceControlAxis(rdk::JointGroup::ARM_1, {!kLockX, !kLockY, !kLockZ, false, false, false},
                    kActiveMaxLinearVel);  // 线速度上限只在主动力控下起作用。
            }
            robot->SetCartesianImpedance(rdk::JointGroup::ARM_1, arm1_float_K, arm1_float_Z);
            floating = true;
            std::cout << "ARM_1 已浮动，能拖的轴 " << AxisNames(false) << "，锁定的轴 " << AxisNames(true)
                      << "：能拖的轴目标跟随实测值，松手停在原地；ARM_2 hold 不动。按 Ctrl+C 结束。" << std::endl;
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
            ++rt_mode_checks;  // 走到这里说明这次查到的控制模式仍是 RT_CARTESIAN_MOTION_FORCE。
            drag_seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - float_start).count();
            const bool follow_null_space = floating && params.null_space_follow;
            if (logging || follow_null_space) {
                const auto arm1_now = robot->states().at(rdk::JointGroup::ARM_1);
                if (logging) {  // 文件读写在主线程，不在实时任务里。
                    WriteLogRow(log, drag_seconds, arm1_now);
                }
                if (follow_null_space) {
                    // 肘部参考姿态 = 当前关节位置，夹在关节范围内（超出范围 SetNullSpacePosture 会报错）。
                    // 阻塞到送达，所以放在主线程，不在实时任务里；记下最长用时。
                    auto q_ref = arm1_now.q;
                    for (size_t i = 0; i < kDoF; ++i) {
                        q_ref[i] = std::clamp(q_ref[i], arm1_q_min[i], arm1_q_max[i]);
                    }
                    const auto call_start = std::chrono::steady_clock::now();
                    robot->SetNullSpacePosture(rdk::JointGroup::ARM_1, q_ref);
                    null_space_max_ms = std::max(null_space_max_ms,
                        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - call_start).count());
                    ++null_space_updates;
                }
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
    if (rt_mode_checks > 0) {
        std::cout << "拖动 " << Num(drag_seconds) << " 秒期间主线程每 " << kMonitorPeriod.count() << " ms 查一次控制模式，"
                  << rt_mode_checks << " 次都是 RT_CARTESIAN_MOTION_FORCE；③ 实时任务共发出 " << g_sent_count.load()
                  << " 次 StreamCartesianMotionForce（实时接口，1 kHz）" << std::endl;
    }
    if (null_space_updates > 0) {
        std::cout << "肘部参考姿态跟随：SetNullSpacePosture 调用 " << null_space_updates << " 次，单次最长 "
                  << Num(null_space_max_ms) << " ms" << std::endl;
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
