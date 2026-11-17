// RT 笛卡尔导纳控制：ARM_1 末端 X Y Z Rx Ry Rz 每个轴由 kLockX … kLockRz 决定能拖还是锁定，
// 能拖的轴按虚拟的质量-阻尼模型跟着手走；ARM_2 全程不动。
// RT Cartesian admittance control: each ARM_1 TCP axis (X Y Z Rx Ry Rz) is draggable or locked, set by kLockX … kLockRz;
// draggable axes follow the hand through a virtual mass-damper model. ARM_2 holds still throughout.
//
// 所附方案（原始版本 third_party/flexiv_rdk/example/intermediate8_realtime_cartesian_floating.cpp，不改它）是降刚度浮动，
// 属于阻抗控制；本文件 11-15 改成导纳控制。
// The attached scheme (original: third_party/flexiv_rdk/example/intermediate8_realtime_cartesian_floating.cpp, left
// untouched) floats the arm by lowering stiffness, i.e. impedance control; this file was switched to admittance on 11-15.
//
// 运行流程照 arm1_transparent_arm2_hold_rt.cpp / Run sequence follows arm1_transparent_arm2_hold_rt.cpp:
//   ① 回正：ARM_1 跑 Home 原语回到 home 姿态（--no-home 跳过）；
//      Home: ARM_1 runs the Home primitive (skip with --no-home).
//   ② 受力回 0：ARM_1 跑 ZeroFTSensor 原语，把末端外力估算清零。导纳的输入就是这个外力，零点不准末端会自己走；
//      Zero force: ARM_1 runs ZeroFTSensor to zero the estimated TCP wrench. That wrench is the admittance input,
//      so a bad zero makes the TCP move on its own.
//   ③ 运动：切到实时模式 RT_CARTESIAN_MOTION_FORCE，ARM_1 导纳控制可以手拖，一直到 Ctrl+C。
//      Motion: switch to RT_CARTESIAN_MOTION_FORCE; ARM_1 is hand-guided under admittance control until Ctrl+C.
// 给 ARM_1 发指令的同时一直给 ARM_2 发 hold：①② 每条原语指令都是 {ARM_1 的原语, ARM_2 Hold}，③ 每次实时指令都同时带两条臂。
// ARM_2 always gets a hold command alongside ARM_1: in ①② every primitive command is {ARM_1 primitive, ARM_2 Hold};
// in ③ every RT command carries both arms.
// Home、ZeroFTSensor 只有原语版本，原语只能在非实时的 NRT_PRIMITIVE_EXECUTION 下运行，控制模式又是整机共用的，
// 所以 ①② 是非实时模式；从 ③ 开始到结束全程实时模式，不再切换。
// Home and ZeroFTSensor exist only as primitives, which run only in NRT_PRIMITIVE_EXECUTION, and the control mode is
// shared by the whole robot, so ①② are non-RT; from ③ to the end the robot stays in RT mode without switching.
//
// ③ 实时调度器每 1 ms 给两条臂各发一次末端指令（StreamCartesianMotionForce），由控制器内置的运动控制器跟踪。
// In ③ the RT scheduler sends both arms a TCP command every 1 ms (StreamCartesianMotionForce), tracked by the
// controller's built-in motion controller.
//   ARM_1：6 轴纯位置控制，刚度一直是名义刚度 K_x_nom、阻尼比 0.7（RDK 默认），不降刚度、不开力控，不自己解 IK。
//          先把目标锁在进入 ③ 时的末端位姿，成功发满 kWarmupCycles 次，确认实时指令一直在发，然后开始导纳：
//          每 1 ms 读末端外力 F = tcp_wrench（世界坐标系，手加在 TCP 上的力和力矩，本机由关节力矩估算），
//          能拖的轴按 M·dv/dt + D·v = F 算出目标速度 v，积分出目标位姿，和目标速度 twist_d 一起发出去，
//          由名义刚度的位置控制跟踪。手感只由 M（kAdmittanceMass）、D（kAdmittanceDamping）决定，和机器人本身的
//          刚度、阻尼无关。锁定的轴目标速度 0、目标位姿保持进入 ③ 时的值，名义刚度锁住。
//   ARM_1: pure 6-axis motion control at nominal stiffness K_x_nom and damping ratio 0.7 (RDK default) throughout;
//          no stiffness reduction, no force control, no IK of our own. The target is first held at the TCP pose on
//          entering ③ until kWarmupCycles commands have gone out, confirming RT streaming works; then admittance
//          starts: every 1 ms read the TCP wrench F = tcp_wrench (world frame; force and moment the hand applies at
//          the TCP, estimated from joint torques on this robot), get each draggable axis's target velocity v from
//          M·dv/dt + D·v = F, integrate it into the target pose, and send both (pose_d, twist_d) to the
//          nominal-stiffness motion controller. The feel is set only by M (kAdmittanceMass) and D
//          (kAdmittanceDamping), not by the robot's own stiffness or damping. Locked axes keep zero target velocity
//          and the entry pose, held at nominal stiffness.
//          和改之前的降刚度浮动不同：那时目标跟随实测位姿、刚度降到 0，手直接推机器人，tcp_wrench 只记录不用；
//          现在 tcp_wrench 是控制输入，机器人本身一直是硬的位置控制。
//          Unlike the earlier reduced-stiffness floating, where the target followed the measured pose at zero
//          stiffness and the hand pushed the robot directly (tcp_wrench was only logged), tcp_wrench is now the
//          control input and the robot itself stays under stiff motion control.
//   ARM_2：6 轴纯位置控制、名义刚度，目标位姿一直是进入 ③ 时的末端位姿（官方 intermediate5 的 --hold）。
//          实时模式下没有 Hold 原语可用，ARM_2 的 hold 指令就是每 1 ms 发一次的这个位姿。
//   ARM_2: pure 6-axis motion control at nominal stiffness, target fixed at its TCP pose on entering ③ (as in the
//          official intermediate5 --hold). RT mode has no Hold primitive; this pose, sent every 1 ms, is the hold.
// 7 轴臂的肘部（零空间）不归末端导纳管，由零空间姿态控制拉回进入 ③ 时的关节姿态；--nullspace-follow 时参考姿态
// 跟随实测关节位置，基本不再拉回。
// The elbow (null space) of the 7-DoF arm is not governed by the TCP admittance: null-space posture control pulls it
// back to the joint posture on entering ③. With --nullspace-follow the reference follows the measured joints instead.
// tcp_wrench 由关节力矩估算：碰肘部等末端以外的地方，估算出的末端外力也会让末端动。
// tcp_wrench is estimated from joint torques, so touching the elbow or other links also moves the TCP.
//
// 编译（项目根目录）/ Build (project root): cmake -S script -B .build/script && cmake --build .build/script
// 运行（实时调度器需要 root）/ Run (the RT scheduler needs root):
//   sudo .build/script/intermediate8_realtime_cartesian_floating <序列号 serial> [选项 options]；--help 看选项 / see --help.

#include <flexiv/rdk/robot.hpp>
#include <flexiv/rdk/scheduler.hpp>
#include <flexiv/rdk/utility.hpp>

#include <Eigen/Geometry>  // AdmittanceStep、TrackError 的四元数运算；RDK 本身依赖 Eigen 3.4。/ Quaternion math; RDK already needs Eigen 3.4.
#include <unistd.h>  // geteuid：检查是否以 root 运行。/ geteuid: check that we run as root.
// 输出用 std::cout 而不是 spdlog：/usr/local/include 里的 fmt 10 会盖住系统 spdlog 1.9 需要的 fmt 8，编不过。
// Output uses std::cout, not spdlog: fmt 10 in /usr/local/include shadows the fmt 8 that system spdlog 1.9 needs.

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
#include <utility>
#include <vector>

using namespace flexiv;

namespace {
constexpr size_t kDoF = rdk::kSerialJointDoF;  // 每条臂的关节数，7。/ Joints per arm, 7.
// ③ 实时任务周期 [s]：AddTask 的 interval 是 1 ms，导纳模型按它离散。
// Period of the ③ RT task [s]: AddTask runs at a 1 ms interval, and the admittance model is discretized with it.
constexpr double kRtPeriod = 0.001;

// ======================== 导纳参数：在这里调 / Admittance parameters: tune here ========================
// 也可以用命令行参数临时覆盖，不用重新编译。都按 X Y Z Rx Ry Rz 排列，世界坐标系；锁定的轴不看这里的值。
// Command-line options override them without recompiling. Order X Y Z Rx Ry Rz, world frame; locked axes ignore them.
// 导纳模型：M·dv/dt + D·v = F。F 是 tcp_wrench 过了死区之后的值，v 是目标速度，积分出目标位姿。
// Admittance model: M·dv/dt + D·v = F, where F is tcp_wrench after the deadband and v is the target velocity,
// integrated into the target pose.
// 11-15 的默认值取最透明的一组：用 compare_transparency.py 分析 logs/ 里的降刚度浮动日志（i8_ns010*、i8_nsfollow、i8_passive*；
// i8_active* 阻尼大得多，不算），拟合 外力 = 阻尼×速度 + 惯量×加速度 + 常数，取各次里的最小值附近。
// The 11-15 defaults were the most transparent set: compare_transparency.py fits force = damping×velocity
// + inertia×acceleration + offset on the reduced-stiffness floating logs in logs/ (i8_ns010*, i8_nsfollow,
// i8_passive*; i8_active* is far more damped and left out), and they sat near the smallest fitted values.
// 透明的代价：起动、刹车都靠手，松手后还会滑一段；抖或漂得受不了就用 --mass、--damping 调大。
// The price of transparency: the hand does all the starting and braking, and the arm coasts after release; raise
// --mass / --damping if it shakes or drifts too much.
// 11-16 真机：上面那组（平移 3 kg、5 N·s/m，转动 0.05 kg·m²、0.1 N·m·s/rad，logs/i8_adm_m3_d5_20261116_174025.csv）
//   手一碰 1.2 s 内就失控：约 2 Hz 越摆越大，外力到 16 N、末端 0.34 m/s、J4 ±0.8 rad/s、J7 1.37 rad/s；平移 5 kg、
//   10 N·s/m 的 3 次都稳。所以默认平移改成 5、10，转动（扭末端主要靠 J7）改成原来的 3 倍 0.15、0.3，时间常数都是 0.5 s。
// On the robot on 11-16 the set above (3 kg, 5 N·s/m translational; 0.05 kg·m², 0.1 N·m·s/rad rotational;
//   logs/i8_adm_m3_d5_20261116_174025.csv) ran away within 1.2 s of a touch: a growing ~2 Hz oscillation, wrench up to
//   16 N, TCP 0.34 m/s, J4 ±0.8 rad/s, J7 1.37 rad/s. The three runs at 5 kg, 10 N·s/m were stable. So the defaults are
//   now 5, 10 translational and 3× the old rotational values, 0.15, 0.3 (twisting the TCP mostly turns J7); both
//   time constants are 0.5 s.
//
// kAdmittanceMass：虚拟质量 M，X Y Z 是质量 [kg]，Rx Ry Rz 是转动惯量 [kg·m²]。越小起动、变向越轻；太小容易抖
//   （tcp_wrench 是估算值，有噪声和滤波滞后），抖了就调大。浮动日志里平移惯量项 2.6–9.8 kg、转动 0.04–0.44 kg·m²。
//   必须 > 0。命令行 --mass=M（只改 X Y Z，Rx Ry Rz 保持这里的值）或 --mass=M1,M2,M3,M4,M5,M6。
// kAdmittanceMass: virtual mass M, X Y Z as mass [kg], Rx Ry Rz as rotational inertia [kg·m²]. Smaller starts and turns
//   more easily; too small tends to shake (tcp_wrench is an estimate with noise and filter lag), so raise it if it does.
//   The floating logs fit 2.6–9.8 kg translational and 0.04–0.44 kg·m² rotational inertia. Must be > 0.
//   Command line: --mass=M (X Y Z only; Rx Ry Rz keep the values here) or --mass=M1,M2,M3,M4,M5,M6.
constexpr std::array<double, rdk::kCartDoF> kAdmittanceMass = {5.0, 5.0, 5.0, 0.15, 0.15, 0.15};
// kAdmittanceDamping：虚拟阻尼 D，X Y Z [N·s/m]，Rx Ry Rz [N·m·s/rad]。一直用力 F 推，最后匀速 (|F| − 死区)/D：越小越轻、
//   越快。松手后速度按时间常数 M/D 衰减到 0，再滑 速度×M/D 那么远停下，不弹回；默认 M/D 平移、转动都是 0.5 s。
//   浮动日志里平移阻尼 4.2–15.7 N·s/m；转动阻尼只有 i8_ns010、i8_nsfollow 拟合出正值 0.14、0.10 N·m·s/rad，其余是负的
//   （转动没有阻尼，转起来要靠手刹住）。必须 > 0。命令行 --damping=D（只改 X Y Z）或 --damping=D1,D2,D3,D4,D5,D6。
// kAdmittanceDamping: virtual damping D, X Y Z [N·s/m], Rx Ry Rz [N·m·s/rad]. A steady push F ends at constant speed
//   (|F| − deadband)/D: smaller is lighter and faster. After release the speed decays with time constant M/D and the
//   arm coasts speed×M/D before it stops, with no spring-back; default M/D is 0.5 s, translational and rotational.
//   The floating logs fit 4.2–15.7 N·s/m translational damping; only i8_ns010 and i8_nsfollow fit positive rotational
//   damping (0.14, 0.10 N·m·s/rad), the rest negative (no rotational damping, the hand had to brake). Must be > 0.
//   Command line: --damping=D (X Y Z only) or --damping=D1,D2,D3,D4,D5,D6.
// 1 个数只改 X Y Z：转动的单位不同，平移的数用到转动上会重 100 倍。11-16 用 --mass=5 --damping=10 时 6 个轴都是 5、10，
//   转动阻尼是默认的 100 倍，扭末端时 J7 几乎不转（23 秒只转了 7–12°，最快 0.15 rad/s）。
// One number sets X Y Z only: the rotation units differ, and a translational value is 100× too heavy for rotation. On
//   11-16, --mass=5 --damping=10 put 5 and 10 on all 6 axes, 100× the default rotational damping, and twisting the
//   TCP barely turned J7 (7–12° in 23 s, at most 0.15 rad/s).
constexpr std::array<double, rdk::kCartDoF> kAdmittanceDamping = {10.0, 10.0, 10.0, 0.3, 0.3, 0.3};
// kForceDeadband、kMomentDeadband：F 的死区，力 [N]、力矩 [N·m]。能拖的轴上的力（力矩）合起来不到这个值就当作 0，超过的
//   部分才进导纳模型，过死区时不跳。死区就是起动要用的最小力：越小越透明，但 tcp_wrench 零点误差超出死区的部分会让末端
//   以 超出量/D 的速度一直走（转动的 D 小，最容易漂）。浮动日志里静止时 |F| 中位数大多 0.02–0.30 N、起动力中位数
//   0.38–0.69 N，静止时 |M| 的 95% 分位 0.01–0.03 N·m（② 清零后、home 附近；离 home 远了零点误差会变大）。没有命令行参数。
// kForceDeadband, kMomentDeadband: deadband on F, force [N] and moment [N·m]. If the combined force (moment) on the
//   draggable axes is below it, F counts as 0; only the excess enters the model, so F does not jump at the edge. It is
//   the least force that starts the arm: smaller is more transparent, but any tcp_wrench zero error beyond it drives
//   the TCP at excess/D (rotation, with its small D, drifts first). In the floating logs |F| at rest has a median of
//   mostly 0.02–0.30 N, the median starting force is 0.38–0.69 N, and the 95th percentile of |M| at rest is
//   0.01–0.03 N·m (after ② and near home; the zero error grows away from home). No command-line option.
constexpr double kForceDeadband = 0.5;
constexpr double kMomentDeadband = 0.05;
static_assert(kForceDeadband >= 0.0 && kMomentDeadband >= 0.0, "死区不能是负数 / deadbands must not be negative");
// kNullSpaceRefTracking：SetNullSpaceObjectives 的 ref_positions_tracking，肘部（零空间）拉回进入 ③ 时关节姿态的权重。
//   范围 0.1–1.0，RDK 默认 0.5；越小肘部越自由，这里取最小值 0.1（最透明），它不等于关掉零空间控制。另外两个权重用 RDK 默认 0。
//   命令行 --nullspace=W。
// kNullSpaceRefTracking: ref_positions_tracking of SetNullSpaceObjectives, the weight pulling the elbow (null space)
//   back to the joint posture on entering ③. Range 0.1–1.0, RDK default 0.5; smaller frees the elbow more. 0.1 is the
//   minimum (most transparent) and does not turn null-space control off. The other two weights keep the RDK default 0.
//   Command line: --nullspace=W.
constexpr double kNullSpaceRefTracking = 0.1;
// kNullSpaceFollow：true = 肘部参考姿态跟随实测关节位置：导纳开始后主线程每 kMonitorPeriod 把 SetNullSpacePosture 设成
//   当前关节位置，零空间几乎不再把肘部拉回，肘部自运动（J1 J2 J3 一起转、末端不动的那种）变轻；代价是没人推时肘部
//   可能自己漂。日志里 i8_nsfollow 和 i8_ns010 互有高低（平移阻尼 9.5 对 7.3 N·s/m，转动 0.10 对 0.14 N·m·s/rad），
//   没有明显更透明，默认不开。命令行 --nullspace-follow。
// kNullSpaceFollow: true = the elbow reference follows the measured joints: once admittance starts, the main thread
//   sets SetNullSpacePosture to the current joint positions every kMonitorPeriod, so the null space barely pulls the
//   elbow back and elbow self-motion (J1 J2 J3 turning together with the TCP still) gets lighter, at the cost of the
//   elbow possibly drifting on its own. In the logs i8_nsfollow and i8_ns010 each win on one axis type (translational
//   damping 9.5 vs 7.3 N·s/m, rotational 0.10 vs 0.14 N·m·s/rad), neither clearly more transparent, so it is off by
//   default. Command line: --nullspace-follow.
constexpr bool kNullSpaceFollow = false;
// kLockX … kLockRz：ARM_1 末端每个轴锁不锁，没有命令行参数，改了要重新编译。
//   true = 锁定：这个轴的目标速度一直是 0、目标一直是进入 ③ 时的值，名义刚度锁住，和 ARM_2 hold 一样硬；
//          这个轴上的外力不进导纳模型。
//   false = 能拖：这个轴按导纳模型跟着手走。
//   都是世界坐标系的轴：RDK 的笛卡尔运动控制一直用世界坐标系，和 tcp_wrench 同一个顺序。
//   转动轴锁定的意思是：从进入 ③ 时的姿态转到目标姿态的旋转向量（世界坐标系）在这个轴上的分量一直是 0，见 AdmittanceStep。
//   默认 6 个轴都能拖（最透明）。
// kLockX … kLockRz: whether each ARM_1 TCP axis is locked; no command-line option, recompile after changing.
//   true = locked: the axis keeps zero target velocity and its value on entering ③, held at nominal stiffness, as
//          stiff as the ARM_2 hold; the wrench on this axis does not enter the admittance model.
//   false = draggable: the axis follows the hand through the admittance model.
//   All are world-frame axes: RDK Cartesian motion control always uses the world frame, in the same order as tcp_wrench.
//   A locked rotation axis means the world-frame rotation vector from the entry orientation to the target orientation
//   keeps a zero component on that axis; see AdmittanceStep.
//   By default all 6 axes are draggable (most transparent).
constexpr bool kLockX = false;
constexpr bool kLockY = false;
constexpr bool kLockZ = false;
constexpr bool kLockRx = false;
constexpr bool kLockRy = false;
constexpr bool kLockRz = false;
// ARM_1 的刚度（名义刚度 K_x_nom）、阻尼比（RDK 默认 0.7）、目标力（0）固定，不是参数：导纳的手感不靠它们调。
// ARM_1 stiffness (nominal K_x_nom), damping ratio (RDK default 0.7) and target wrench (0) are fixed, not parameters:
// the admittance feel is not tuned through them.
// ====================================================================================

// 程序内部用，由上面 6 个参数得出，不用改：按 X Y Z Rx Ry Rz 排列。
// Derived from the 6 flags above for internal use; do not edit. Order X Y Z Rx Ry Rz.
constexpr std::array<bool, rdk::kCartDoF> kLockAxis = {kLockX, kLockY, kLockZ, kLockRx, kLockRy, kLockRz};
const std::array<std::string, rdk::kCartDoF> kAxisNames = {"X", "Y", "Z", "Rx", "Ry", "Rz"};
constexpr bool kAnyRotationFree = !kLockRx || !kLockRy || !kLockRz;  // Rx Ry Rz 里至少一个能拖。/ At least one rotation axis is draggable.
constexpr bool kAllRotationFree = !kLockRx && !kLockRy && !kLockRz;  // Rx Ry Rz 三个都能拖。/ All three rotation axes are draggable.

// ===== ③ ARM_2 hold：发给机器人，只影响 ARM_2，不影响 ARM_1 手感 / Sent to the robot; affects ARM_2 only, not ARM_1's feel =====
// kArm2NullSpaceRefTracking：ARM_2 的 SetNullSpaceObjectives ref_positions_tracking，肘部拉回 hold 姿态的权重。
//   范围 0.1–1.0，RDK 默认 0.5；取上限 1.0，拉得最紧。11-03 实测默认 0.5 时 ARM_2 末端不动，但 J1、J3 反向转
//   （J2 ≈ -0.09，J1 J3 近乎共轴，是零空间自运动），约 4 秒超过当时 0.01 rad 的停机检查。另外两个权重用 RDK 默认 0。
// kArm2NullSpaceRefTracking: ARM_2's ref_positions_tracking, the weight pulling its elbow back to the hold posture.
//   Range 0.1–1.0, RDK default 0.5; set to the maximum 1.0, pulling hardest. On 11-03, at the default 0.5, the ARM_2 TCP
//   stayed still but J1 and J3 turned in opposite directions (J2 ≈ -0.09, so J1 and J3 are nearly coaxial: null-space
//   self-motion) and tripped the then 0.01 rad stop check in about 4 s. The other two weights keep the RDK default 0.
constexpr double kArm2NullSpaceRefTracking = 1.0;

// ===== ① 回正、② 受力回 0：原语参数，同 arm1_transparent_arm2_hold_rt.cpp，照抄 Flexiv Primitives 3E.1 文档，不影响手感 =====
// ===== ① Home, ② zero force: primitive parameters, same as arm1_transparent_arm2_hold_rt.cpp, copied from the
//       Flexiv Primitives 3E.1 docs; they do not affect the feel =====
// Home 的 jntVelScale：关节速度百分比，100 为全速。范围 1–100，文档默认 20。
// Home jntVelScale: joint speed in percent, 100 = full speed. Range 1–100, doc default 20.
constexpr int kHomeJntVelScale = 20;
// Home 的 target 不传，用文档默认的 home 姿态 0 -40 0 90 0 -50 0 deg。
// Home's target is not passed, so the doc default home posture 0 -40 0 90 0 -50 0 deg is used.
// ZeroFTSensor（Basic Force Control 页）：清零时机器人要静止，不能被碰、不能接触环境。程序等 ARM_1 回正停稳后才清零。
// ZeroFTSensor (Basic Force Control page): the robot must be still, untouched and out of contact while zeroing.
// The program zeroes only after ARM_1 has homed and stopped.
constexpr double kZeroDataCollectTime = 0.2;  // dataCollectTime：采样多久来定零点 [s]，范围 0.1–1.0，文档默认 0.2。/ Sampling time for the zero [s], range 0.1–1.0, doc default 0.2.
constexpr int kZeroEnableStaticCheck = 0;  // enableStaticCheck：清零前检查是否静止，1 检查，文档默认 0。/ Check stillness before zeroing, 1 = check, doc default 0.
constexpr int kZeroCalibExtraPayload = 0;  // calibExtraPayload：额外负载算不算进工具负载，1 算，文档默认 0。/ Count extra payload into the tool payload, 1 = yes, doc default 0.

// ========== 停机条件和流程：不发给机器人、不影响手感，只决定什么时候整机停 ==========
// ========== Stop conditions and sequencing: not sent to the robot, no effect on feel; they only decide when to stop ==========
// 11-03 按要求不设速度、位置限位类的停机检查：ARM_1 末端速度、离起点的位移、关节离极限的余量、关节速度，
// ARM_2 偏离 hold 位置，都不检查（ARM_2 的偏离仍然记录，结束时打印）。
// As requested on 11-03 there are no speed- or position-limit stop checks: ARM_1 TCP speed, displacement from the start,
// joint-limit margin, joint speed and ARM_2's deviation from hold are not checked (ARM_2's deviation is still recorded
// and printed at the end).
// 只在机器人自己不能继续时停：断线、E-stop、故障、未就绪、控制模式被切走、实时指令迟到过多、实时任务心跳丢失、
// ARM_1 跟不上导纳目标、tcp_wrench 不是有限值。
// The program stops only when the robot cannot go on: disconnection, E-stop, fault, not operational, control mode
// switched away, too many late RT commands, lost RT heartbeat, ARM_1 unable to follow the admittance target, or a
// non-finite tcp_wrench.
// 控制器自带的保护（关节硬极限、安全设置里的限速、碰撞检测等）RDK 关不掉，照常有效。
// The controller's own protections (hard joint limits, safety speed limits, collision detection, etc.) cannot be
// disabled through RDK and stay active.
// 安全：③ 实时任务这么久没成功发出指令就整机停。/ Safety: stop if the ③ RT task sends nothing successfully for this long.
constexpr auto kHeartbeatTimeout = std::chrono::milliseconds(100);
// 安全：SetTimelinessFailureLimit，1 秒内迟到指令的百分比上限，RDK 默认 2。
// Safety: SetTimelinessFailureLimit, maximum percentage of late commands within 1 s, RDK default 2.
constexpr double kTimelinessFailureLimit = 2.0;
// 安全：导纳开始后 ARM_1 实测位姿离上一次发出的目标位姿超过这么多就整机停。目标是导纳模型积分出来的，不看机器人在哪：
//   机器人被挡住、到关节极限、被控制器限速时跟不上，目标还会接着往前走，名义刚度推得越来越狠。不是速度、位置限位，
//   拖得多快、多远都不查。结束时打印 ③ 期间的最大跟踪误差，按实测再调。
// Safety: once admittance runs, stop if ARM_1's measured pose is this far from the last commanded target. The target is
//   integrated by the admittance model regardless of where the robot is: if the robot is blocked, at a joint limit or
//   speed-limited by the controller, the target keeps moving and the nominal stiffness pushes ever harder. This is not
//   a speed or position limit; how fast or far the arm is dragged is not checked. The maximum tracking error during ③
//   is printed at the end; adjust these from measurements.
constexpr double kMaxTrackPositionError = 0.01;  // [m]，名义刚度 15000 N/m 下约 150 N。/ About 150 N at the nominal 15000 N/m.
constexpr double kMaxTrackAngleError = 0.03;  // [rad]，名义刚度 2000 N·m/rad 下约 60 N·m。/ About 60 N·m at the nominal 2000 N·m/rad.
constexpr uint64_t kWarmupCycles = 100;  // 流程：③ 先锁在起始位姿成功发满这么多次指令，才开始导纳。/ Commands sent at the start pose before admittance begins.
constexpr auto kWarmupTimeout = std::chrono::seconds(2);  // 流程：发满 kWarmupCycles 次的最长时间。/ Time allowed for kWarmupCycles.
constexpr double kOperationalTimeout = 30.0;  // 流程：上伺服后等待就绪的最长时间 [s]。/ Max wait for operational after servo-on [s].
constexpr double kHomeTimeout = 30.0;  // 流程：① 回正到位并停稳的最长时间 [s]。/ Max time for ① to reach home and stop [s].
constexpr double kZeroTimeout = 10.0;  // 流程：② 受力回 0 结束的最长时间 [s]。/ Max time for ② to finish [s].
// 流程：主线程检查断线、E-stop、故障、控制模式、心跳的周期；--log 也按这个周期记一行（100 Hz）。
// Main-thread period for checking connection, E-stop, fault, control mode and heartbeat; --log writes one row per period (100 Hz).
constexpr auto kMonitorPeriod = std::chrono::milliseconds(10);

struct Params  // 命令行覆盖之后实际使用的参数。/ Parameters in effect after command-line overrides.
{
    std::string robot_sn;
    std::array<double, rdk::kCartDoF> mass = kAdmittanceMass;
    std::array<double, rdk::kCartDoF> damping = kAdmittanceDamping;
    double null_space_tracking = kNullSpaceRefTracking;
    bool null_space_follow = kNullSpaceFollow;
    std::string log_path;  // 空 = 不记录。/ Empty = no log.
    bool home = true;
    bool clear_fault = false;
};

struct RtRecord  // ③ 期间的最大值：实时任务写，调度器停下后主线程读。/ Maxima over ③: written by the RT task, read by main after the scheduler stops.
{
    std::array<double, kDoF> arm2_max_error = {};  // ARM_2 偏离 hold 位置 [rad]。/ ARM_2 deviation from hold [rad].
    std::array<double, kDoF> arm2_max_vel = {};  // ARM_2 速度 [rad/s]。/ ARM_2 joint speed [rad/s].
    double arm1_max_position_error = 0.0;  // 导纳开始后 ARM_1 实测离目标的位置 [m]。/ ARM_1 position tracking error under admittance [m].
    double arm1_max_angle_error = 0.0;  // 导纳开始后 ARM_1 实测离目标的姿态 [rad]。/ ARM_1 orientation tracking error under admittance [rad].
};

// ③ 的指令和导纳模型：进入 ③ 时定好，调度器启动后只有实时任务读写，主线程不再碰。
// Commands and admittance model for ③: set up on entering ③; once the scheduler starts only the RT task touches them.
struct RtTargets
{
    // 每 1 ms 发出的两条臂指令。g_admittance 之后实时任务每次用 AdmittanceStep 更新 ARM_1 的 pose_d、twist_d：
    // 它们就是导纳模型的状态（目标位姿、目标速度）。其余不变。
    // Commands for both arms, sent every 1 ms. After g_admittance the RT task updates ARM_1's pose_d and twist_d with
    // AdmittanceStep each cycle: they are the admittance model's state (target pose, target velocity). The rest is fixed.
    std::map<rdk::JointGroup, rdk::RtCartesianCmd> commands;
    // 进入 ③ 时 ARM_1 的末端位姿：锁定的轴一直用它。/ ARM_1 TCP pose on entering ③; locked axes keep it.
    std::array<double, rdk::kPoseSize> arm1_start_pose = {};
    // 导纳模型每个轴按 1 ms 精确离散（F 在 1 ms 内不变）：v ← decay·v + gain·F，decay = exp(−D/M·dt)，gain = (1 − decay)/D。
    // M、D 取多大都不会数值发散。
    // Exact 1 ms discretization per axis (F held over the step): v ← decay·v + gain·F, decay = exp(−D/M·dt),
    // gain = (1 − decay)/D. Numerically stable for any M and D.
    std::array<double, rdk::kCartDoF> decay = {};
    std::array<double, rdk::kCartDoF> gain = {};
    // ARM_2 hold 时的关节位置 [rad]，只用来记录 ARM_2 动了多少，不停机。/ ARM_2 hold joint positions [rad], only to record how far it moved.
    std::vector<double> arm2_q;
};

// 置 true 后实时任务不再发指令，主线程收尾。信号处理函数只做这一件事。
// Once true the RT task stops sending and main shuts down. This is all the signal handler does.
std::atomic<bool> g_stop = {false};
std::mutex g_reason_mutex;  // 保护 g_reason：实时线程和主线程都可能写。/ Guards g_reason; both the RT and main threads may write it.
std::string g_reason;  // 第一个停机原因；空 = Ctrl+C 正常结束。/ First stop reason; empty = normal Ctrl+C exit.
std::atomic<uint64_t> g_sent_count = {0};  // ③ 实时任务成功发出指令的次数。/ Commands the ③ RT task sent successfully.
std::atomic<int64_t> g_last_sent_ns = {0};  // ③ 实时任务最近一次成功发出指令的时刻 [ns]，见 NowNs。/ Time of the last successful send [ns], see NowNs.
// ③ 预热完主线程置 true：之后 ARM_1 的目标每 1 ms 按导纳模型更新。
// Set by main after the ③ warm-up: from then on ARM_1's target is updated by the admittance model every 1 ms.
std::atomic<bool> g_admittance = {false};

void PrintHelp()
{
    std::cout << "用法：sudo .build/script/intermediate8_realtime_cartesian_floating <序列号> [选项]\n"
                 "  <序列号>          完整序列号，不要有空格，例如 Enlight-LL-123456\n"
                 "  --mass=M          覆盖 kAdmittanceMass：ARM_1 能拖的轴（kLockX … kLockRz 为 false）的虚拟质量 > 0，\n"
                 "                    X Y Z [kg]、Rx Ry Rz [kg·m²]；越小起动越轻，太小会抖。\n"
                 "                    1 个数 = 只改 X Y Z，Rx Ry Rz 保持 kAdmittanceMass；\n"
                 "                    逗号分开的 6 个数 = X,Y,Z,Rx,Ry,Rz 各一个，例如 --mass=5,5,5,0.3,0.3,0.3\n"
                 "  --damping=D       覆盖 kAdmittanceDamping：虚拟阻尼 > 0，X Y Z [N·s/m]、Rx Ry Rz [N·m·s/rad]；\n"
                 "                    一直用力 F 推最后匀速 (F − 死区)/D，越小越轻。1 个数（只改 X Y Z）或逗号分开的 6 个数，同 --mass\n"
                 "  --nullspace=W     覆盖 kNullSpaceRefTracking：肘部拉回参考姿态的权重 [0.1, 1]\n"
                 "  --nullspace-follow\n"
                 "                    覆盖 kNullSpaceFollow：肘部参考姿态每 10 ms 跟随实测关节位置，肘部不再被拉回；\n"
                 "                    没人推时肘部可能自己漂\n"
                 "  --log=FILE.csv    ③ 导纳期间按 100 Hz 记录 ARM_1 的末端位姿、速度、外力和关节位置、速度；\n"
                 "                    FILE.csv 已经存在时不运行，不覆盖旧记录，文件名建议带日期时间\n"
                 "  --no-home         不先回正，在当前姿态受力回 0 后直接开始\n"
                 "  --clear-fault     启动时若有故障先尝试清除；不加则直接退出\n"
                 "  -h, --help        显示本说明\n";
}

std::string Num(double value)  // 数字转成最短的文字，例如 10、0.05。/ Shortest text for a number, e.g. 10, 0.05.
{
    std::ostringstream oss;
    oss << value;
    return oss.str();
}

// kLockAxis 里等于 locked 的轴，例如 "X Y Z"；一个都没有时是 "无"。
// Axes whose kLockAxis equals locked, e.g. "X Y Z"; "无" (none) if there are none.
std::string AxisNames(bool locked)
{
    std::string text;
    for (size_t i = 0; i < rdk::kCartDoF; ++i) {
        if (kLockAxis[i] == locked) {
            text += (text.empty() ? "" : " ") + kAxisNames[i];
        }
    }
    return text.empty() ? "无" : text;
}

bool ParseNumber(const char* text, double& value)  // 整个字符串是一个有限的数才算成功。/ Succeeds only if the whole string is one finite number.
{
    char* end = nullptr;
    value = std::strtod(text, &end);
    return end != text && *end == '\0' && std::isfinite(value);
}

// text 是 1 个数（只改 X Y Z，Rx Ry Rz 不变）或逗号分开的 6 个数（X Y Z Rx Ry Rz 各一个）才算成功。
// Succeeds only if text is 1 number (sets X Y Z, leaves Rx Ry Rz) or 6 comma-separated numbers (one per X Y Z Rx Ry Rz).
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
    if (parsed.size() == 1) {
        std::fill(values.begin(), values.begin() + 3, parsed[0]);
    } else {
        std::copy(parsed.begin(), parsed.end(), values.begin());
    }
    return true;
}

// arg 是 "name=值" 时取出值。/ Extracts the value when arg is "name=value".
bool OptionValue(const std::string& arg, const std::string& name, std::string& value)
{
    if (!arg.starts_with(name + "=")) {
        return false;
    }
    value = arg.substr(name.size() + 1);
    return true;
}

// 读取命令行；未给出的用上方常量。出错返回原因。/ Reads the command line; unset options use the constants above. Returns the error, if any.
std::string ParseArgs(int argc, char* argv[], Params& params)
{
    params.robot_sn = argv[1];
    if (params.robot_sn.starts_with("-")) {
        return "第一个参数必须是序列号";
    }
    for (int i = 2; i < argc; ++i) {
        const std::string arg = argv[i];
        std::string value;
        if (OptionValue(arg, "--mass", value)) {
            if (!ParseAxisValues(value, params.mass)) {
                return "--mass= 后面要跟 1 个数或逗号分开的 6 个数";
            }
        } else if (OptionValue(arg, "--damping", value)) {
            if (!ParseAxisValues(value, params.damping)) {
                return "--damping= 后面要跟 1 个数或逗号分开的 6 个数";
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
    // 下发前检查，上方常量也一样要过这一关。/ Checked before use; the constants above must pass too.
    for (size_t i = 0; i < rdk::kCartDoF; ++i) {
        if (!(params.mass[i] > 0.0)) {
            return "mass 的 " + kAxisNames[i] + " = " + Num(params.mass[i]) + "，要大于 0";
        }
        if (!(params.damping[i] > 0.0)) {
            return "damping 的 " + kAxisNames[i] + " = " + Num(params.damping[i]) + "，要大于 0";
        }
    }
    if (!(params.null_space_tracking >= 0.1 && params.null_space_tracking <= 1.0)) {
        return "null_space_tracking = " + Num(params.null_space_tracking) + " 超出范围 [0.1, 1]";
    }
    return "";
}

// 记下第一个停机原因并要求停机；实时线程和主线程都会调用。/ Records the first stop reason and requests a stop; called by both threads.
void RequestStop(const std::string& reason)
{
    std::lock_guard<std::mutex> lock(g_reason_mutex);
    if (g_reason.empty()) {
        g_reason = reason;
    }
    g_stop = true;
}

void OnSignal(int)  // Ctrl+C、关闭终端、kill 都走同一个收尾流程。/ Ctrl+C, closing the terminal and kill share one shutdown path.
{
    g_stop = true;
}

// q − target [rad]。
std::array<double, kDoF> Error(const std::vector<double>& q, const std::vector<double>& target)
{
    std::array<double, kDoF> error;
    for (size_t i = 0; i < kDoF; ++i) {
        error[i] = q[i] - target[i];
    }
    return error;
}

// steady_clock 的当前时刻 [ns]：实时线程记录、主线程比较，用来检查心跳。
// Current steady_clock time [ns]: recorded by the RT thread, compared by main to check the heartbeat.
int64_t NowNs()
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

// RDK 位姿 [x y z qw qx qy qz] 的姿态，归一化。/ Orientation of an RDK pose [x y z qw qx qy qz], normalized.
Eigen::Quaterniond Orientation(const std::array<double, rdk::kPoseSize>& pose)
{
    return Eigen::Quaterniond(pose[3], pose[4], pose[5], pose[6]).normalized();
}

// 实测位姿 measured 离目标位姿 target 的位置误差 [m] 和姿态误差（转角）[rad]。
// Position error [m] and orientation error (rotation angle) [rad] of the measured pose from the target pose.
std::pair<double, double> TrackError(
    const std::array<double, rdk::kPoseSize>& measured, const std::array<double, rdk::kPoseSize>& target)
{
    const Eigen::Vector3d position_error(
        measured[0] - target[0], measured[1] - target[1], measured[2] - target[2]);
    return {position_error.norm(), Orientation(measured).angularDistance(Orientation(target))};
}

// 导纳模型往前走 1 ms：按 ARM_1 末端外力 wrench 更新目标速度 cmd.twist_d、目标位姿 cmd.pose_d。
// 能拖的轴 M·dv/dt + D·v = F；锁定的轴速度 0、目标保持进入 ③ 时的位姿。
// 位姿是 RDK 的 [x y z qw qx qy qz]，速度、外力是 [平移 3 个, 转动 3 个]，都在世界坐标系。
// 每 1 ms 在实时任务里调用，只用固定大小的数据，不分配内存。
// Advances the admittance model by 1 ms: updates the target velocity cmd.twist_d and target pose cmd.pose_d from the
// ARM_1 TCP wrench. Draggable axes follow M·dv/dt + D·v = F; locked axes keep zero velocity and the entry pose.
// Poses are RDK [x y z qw qx qy qz]; velocities and wrenches are [3 linear, 3 angular]; all in the world frame.
// Called every 1 ms in the RT task; uses fixed-size data only and does not allocate.
void AdmittanceStep(const std::array<double, rdk::kCartDoF>& wrench, const RtTargets& targets, rdk::RtCartesianCmd& cmd)
{
    // F：去掉锁定的轴，力、力矩各自按合起来的大小过死区，方向不变。
    // F: drop locked axes, then apply the deadband to the force and moment magnitudes separately, keeping direction.
    std::array<double, rdk::kCartDoF> force = {};
    for (size_t i = 0; i < rdk::kCartDoF; ++i) {
        force[i] = kLockAxis[i] ? 0.0 : wrench[i];
    }
    for (const size_t first : {size_t {0}, size_t {3}}) {
        const double deadband = first == 0 ? kForceDeadband : kMomentDeadband;
        const double norm = std::hypot(force[first], force[first + 1], force[first + 2]);
        const double keep = norm > deadband ? (norm - deadband) / norm : 0.0;
        for (size_t i = first; i < first + 3; ++i) {
            force[i] *= keep;
        }
    }

    auto& twist = cmd.twist_d;
    for (size_t i = 0; i < rdk::kCartDoF; ++i) {
        twist[i] = kLockAxis[i] ? 0.0 : targets.decay[i] * twist[i] + targets.gain[i] * force[i];
    }
    // 位置：锁定的轴速度是 0，一直是进入 ③ 时的值。/ Position: locked axes have zero velocity and keep the entry value.
    for (size_t i = 0; i < 3; ++i) {
        cmd.pose_d[i] += twist[i] * kRtPeriod;
    }
    if (!kAnyRotationFree) {  // Rx Ry Rz 都锁定：姿态一直是进入 ③ 时的值。/ All rotations locked: keep the entry orientation.
        return;
    }

    // 姿态：按世界坐标系的角速度 ω 转 1 ms，q_d ← exp(ω·dt) ⊗ q_d。
    // Orientation: rotate by the world-frame angular velocity ω for 1 ms, q_d ← exp(ω·dt) ⊗ q_d.
    const Eigen::Quaterniond q_prev = Orientation(cmd.pose_d);
    Eigen::Quaterniond q_d = q_prev;
    const Eigen::Vector3d omega(twist[3], twist[4], twist[5]);
    if (const double rate = omega.norm(); rate > 0.0) {
        q_d = Eigen::Quaterniond(Eigen::AngleAxisd(rate * kRtPeriod, omega / rate)) * q_d;
    }
    if (!kAllRotationFree) {
        // 只锁一部分转动轴：锁定轴的 ω 已经是 0，但绕两个世界轴的转动叠起来，第三个轴上也会带出一点。把从起始姿态转到
        // 目标姿态的旋转向量 r（世界坐标系）在锁定轴上的分量清零，目标姿态 = exp(r) ⊗ 起始姿态。
        // Some rotations locked: their ω is already 0, but rotations about two world axes compound into a little rotation
        // about the third. Zero the locked components of the world-frame rotation vector r from the start orientation to
        // the target, then target = exp(r) ⊗ start.
        const Eigen::Quaterniond q_start = Orientation(targets.arm1_start_pose);
        const Eigen::AngleAxisd rotation(q_d * q_start.conjugate());
        Eigen::Vector3d r = rotation.angle() * rotation.axis();
        for (Eigen::Index i = 0; i < 3; ++i) {
            if (kLockAxis[static_cast<size_t>(3 + i)]) {
                r[i] = 0.0;
            }
        }
        q_d = q_start;
        if (const double angle = r.norm(); angle > 0.0) {
            q_d = Eigen::Quaterniond(Eigen::AngleAxisd(angle, r / angle)) * q_start;
        }
    }
    q_d.normalize();
    // q 和 −q 是同一个姿态，取和上一次同号的那个，目标四元数不跳。/ q and −q are the same orientation; keep the sign of the last target so it does not jump.
    if (q_d.dot(q_prev) < 0.0) {
        q_d.coeffs() *= -1.0;
    }
    cmd.pose_d[3] = q_d.w();
    cmd.pose_d[4] = q_d.x();
    cmd.pose_d[5] = q_d.y();
    cmd.pose_d[6] = q_d.z();
}

// 实时任务：每 1 ms 记录一次 ARM_2 的偏离（不停机）；导纳开始后检查 ARM_1 跟不跟得上、按外力更新 ARM_1 的目标；
// 然后把两条臂的末端指令放进同一条 StreamCartesianMotionForce 发出去。
// RT task, every 1 ms: record ARM_2's deviation (no stop); under admittance, check that ARM_1 keeps up and update its
// target from the wrench; then send both arms' TCP commands in one StreamCartesianMotionForce call.
void PeriodicTask(rdk::Robot& robot, RtTargets& targets, RtRecord& record)
{
    if (g_stop) {
        return;  // 要停了：不再发指令，等主线程停掉调度器后 Stop()。/ Stopping: send nothing more; main stops the scheduler, then calls Stop().
    }
    try {
        if (robot.fault()) {
            throw std::runtime_error("机器人故障");
        }
        const auto all_states = robot.states();
        const auto& arm1 = all_states.at(rdk::JointGroup::ARM_1);
        const auto& arm2 = all_states.at(rdk::JointGroup::ARM_2);
        const auto arm2_error = Error(arm2.q, targets.arm2_q);  // ARM_2 离 hold 位置的偏离 [rad]。/ ARM_2 deviation from hold [rad].
        for (size_t i = 0; i < kDoF; ++i) {
            record.arm2_max_error[i] = std::max(record.arm2_max_error[i], std::abs(arm2_error[i]));
            record.arm2_max_vel[i] = std::max(record.arm2_max_vel[i], std::abs(arm2.dtheta[i]));
        }

        // ARM_1：预热时目标是进入 ③ 时的位姿；导纳开始后先看上一次发出的目标跟上没有，再按这次的外力往前走 1 ms。
        // ARM_2：进入 ③ 时的位姿，就是 hold。两条臂每次都一起发。
        // ARM_1: during warm-up the target is the entry pose; under admittance, first check that the last target was
        // tracked, then advance the model by 1 ms with this cycle's wrench.
        // ARM_2: the entry pose, i.e. the hold. Both arms are always sent together.
        if (g_admittance) {
            auto& arm1_cmd = targets.commands.at(rdk::JointGroup::ARM_1);
            const auto [position_error, angle_error] = TrackError(arm1.tcp_pose, arm1_cmd.pose_d);
            record.arm1_max_position_error = std::max(record.arm1_max_position_error, position_error);
            record.arm1_max_angle_error = std::max(record.arm1_max_angle_error, angle_error);
            // 写成 !(… <= …)，误差是 NaN 时也停。/ Written as !(… <= …) so a NaN error stops too.
            if (!(position_error <= kMaxTrackPositionError && angle_error <= kMaxTrackAngleError)) {
                throw std::runtime_error("ARM_1 跟不上导纳目标：位置差 " + Num(position_error * 1000.0) + " mm、姿态差 "
                                         + Num(angle_error) + " rad（上限 " + Num(kMaxTrackPositionError * 1000.0)
                                         + " mm、" + Num(kMaxTrackAngleError) + " rad）");
            }
            for (double w : arm1.tcp_wrench) {
                if (!std::isfinite(w)) {
                    throw std::runtime_error("ARM_1 tcp_wrench 不是有限值");
                }
            }
            AdmittanceStep(arm1.tcp_wrench, targets, arm1_cmd);
        }
        robot.StreamCartesianMotionForce(targets.commands);
        g_last_sent_ns = NowNs();
        ++g_sent_count;
    } catch (const std::exception& e) {
        RequestStop(e.what());
    }
}

// 机器人不能继续在 mode 下运行的原因；正常返回空串。/ Why the robot cannot keep running in mode; empty if it can.
std::string NotRunningReason(const rdk::Robot& robot, rdk::Mode mode)
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

// ③ 实时任务超过 kHeartbeatTimeout 没有成功发出指令的原因；正常返回空串。
// Why the ③ RT task has sent nothing for over kHeartbeatTimeout; empty if it is fine.
std::string HeartbeatLost()
{
    const auto age = std::chrono::nanoseconds(NowNs() - g_last_sent_ns);
    if (age <= kHeartbeatTimeout) {
        return "";
    }
    return "实时任务 " + Num(std::chrono::duration<double, std::milli>(age).count()) + " ms 没有发出指令（kHeartbeatTimeout = "
           + Num(static_cast<double>(kHeartbeatTimeout.count())) + " ms）";
}

// 连接后的检查和上伺服；不满足就抛出，此时还没发过运动指令。
// Post-connection checks and servo-on; throws if anything is wrong, before any motion command has been sent.
void Prepare(rdk::Robot& robot, const Params& params)
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
    // 实时模式下外部轴也要每 1 ms 给指令，这里不处理。/ In RT mode an external axis would also need commands every 1 ms; not handled here.
    if (info.all_groups.contains(rdk::JointGroup::EXT_AXIS)) {
        throw std::runtime_error("机器人有外部轴；本程序只给 ARM_1、ARM_2 发指令");
    }
    if (!robot.all_stopped()) {  // 有别的程序在动机器人时不接管。/ Do not take over while another program moves the robot.
        throw std::runtime_error("机器人还在运动；先停止其他控制程序");
    }
    if (!robot.operational()) {
        robot.ServoOn();  // 上伺服；E-stop 必须已松开。/ Servo on; the E-stop must be released.
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
// For ①②: ARM_1 runs arm1_primitive and ARM_2 runs Hold in one ExecutePrimitive call, then check every kMonitorPeriod
// until done returns true. done gets ARM_1's primitive states, or nullptr once that primitive is no longer running.
// Throws on timeout, when the robot cannot run, or on Ctrl+C.
void RunArm1Primitive(rdk::Robot& robot, const rdk::PrimitiveArgs& arm1_primitive, double timeout,
    const std::function<bool(const rdk::PrimitiveStates*)>& done)
{
    std::map<rdk::JointGroup, rdk::PrimitiveArgs> commands = {
        {rdk::JointGroup::ARM_1, arm1_primitive},
        {rdk::JointGroup::ARM_2, rdk::PrimitiveArgs("Hold", {})},  // Hold 没有输入参数。/ Hold takes no inputs.
    };
    for (auto& entry : commands) {
        entry.second.external_axis_control = false;  // 不带动外部轴。/ Do not drive external axes.
        entry.second.sync_motion_mode = rdk::SyncMotionMode::DISABLE;  // 两臂互不跟随。/ The arms do not follow each other.
    }
    robot.ExecutePrimitive(commands);  // 阻塞到两条原语开始执行。/ Blocks until both primitives start.
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout);
    while (true) {
        if (g_stop) {
            throw std::runtime_error(arm1_primitive.pt_name + " 期间被中断");
        }
        if (const auto reason = NotRunningReason(robot, rdk::Mode::NRT_PRIMITIVE_EXECUTION); !reason.empty()) {
            throw std::runtime_error(reason);
        }
        const auto states = robot.primitive_states();  // 只包含正在运行原语的关节组。/ Only groups with a running primitive.
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

// 打印 ARM_1 外力矩和末端外力，对比受力回 0 前后。/ Prints ARM_1's external joint torques and TCP wrench, before and after zeroing.
void PrintArm1Force(const rdk::Robot& robot, const std::string& when)
{
    const auto arm1 = robot.states().at(rdk::JointGroup::ARM_1);
    std::cout << "ARM_1 受力回 0 " << when << "：外力矩 tau_ext [" << rdk::utility::Vec2Str(arm1.tau_ext, 2)
              << "] Nm，末端外力 tcp_wrench [" << rdk::utility::Arr2Str(arm1.tcp_wrench, 2) << "] N/Nm" << std::endl;
}

// ③ 开始导纳之前：等实时任务锁在起始位姿成功发满 kWarmupCycles 次指令。被 Ctrl+C 打断或实时任务已要求停机时返回 false。
// Before ③ admittance: wait until the RT task has sent kWarmupCycles commands at the start pose. Returns false on
// Ctrl+C or if the RT task already requested a stop.
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

// --log 的表头：时间、ARM_1 末端位姿、速度、外力，关节位置、速度。
// --log header: time, ARM_1 TCP pose, twist, wrench, joint positions and velocities.
void WriteLogHeader(std::ostream& out)
{
    out << "t_s,x,y,z,qw,qx,qy,qz,vx,vy,vz,wx,wy,wz,fx,fy,fz,mx,my,mz";
    for (const char* name : {"q", "dtheta"}) {
        for (size_t i = 1; i <= kDoF; ++i) {
            out << ',' << name << i;
        }
    }
    out << '\n';
}

// --log 的一行，t 是导纳开始后的秒数。末端位姿、速度、外力都在世界坐标系；外力 tcp_wrench 是 ② 清零之后的读数。
// One --log row; t is seconds since admittance started. Pose, twist and wrench are in the world frame; tcp_wrench is
// the reading after the ② zeroing.
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
    if (geteuid() != 0) {  // RDK README：用到 rdk::Scheduler 的程序必须以 root 运行。/ RDK README: programs using rdk::Scheduler must run as root.
        std::cerr << "实时调度器需要 root 权限：sudo " << argv[0] << " ...\n";
        return 2;
    }
    std::ofstream log;  // 连机器人之前打开：路径不对时机器人还没动就退出。/ Opened before connecting, so a bad path exits before the robot moves.
    if (!params.log_path.empty()) {
        // 查不了（例如没有权限）就当不存在，下面 open 失败时照样退出。/ If existence cannot be checked (e.g. permissions), assume absent; a failed open still exits below.
        std::error_code exists_error;
        if (std::filesystem::exists(params.log_path, exists_error)) {  // 不覆盖以前的记录。/ Never overwrite old logs.
            std::cerr << "日志文件 " << params.log_path << " 已经存在，为了不覆盖以前的记录，程序不运行。\n"
                         "换个文件名，例如带日期时间：--log=logs/i8_adm_$(date +%Y%m%d_%H%M%S).csv\n";
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
    // Declaration order sets destruction order: the scheduler is destroyed first, so the RT task never sees a destroyed
    // robot or targets.
    std::unique_ptr<rdk::Robot> robot;
    RtTargets targets;
    RtRecord rt_record;
    std::unique_ptr<rdk::Scheduler> scheduler;
    bool need_stop = false, started = false, logging = false, admitting = false;
    // 拖动期间主线程查到控制模式是 RT_CARTESIAN_MOTION_FORCE 的次数，结束时打印。/ Times main saw RT_CARTESIAN_MOTION_FORCE while dragging, printed at the end.
    uint64_t rt_mode_checks = 0;
    uint64_t null_space_updates = 0;  // --nullspace-follow：SetNullSpacePosture 调用次数，结束时打印。/ SetNullSpacePosture calls, printed at the end.
    double null_space_max_ms = 0.0;  // --nullspace-follow：SetNullSpacePosture 单次最长用时 [ms]。/ Longest single SetNullSpacePosture call [ms].
    double drag_seconds = 0.0;  // 拖动（ARM_1 导纳开始之后）持续的秒数。/ Seconds of dragging (since ARM_1 admittance started).
    for (size_t i = 0; i < rdk::kCartDoF; ++i) {  // 导纳模型的离散系数，见 RtTargets。/ Discretization coefficients, see RtTargets.
        targets.decay[i] = std::exp(-params.damping[i] / params.mass[i] * kRtPeriod);
        targets.gain[i] = (1.0 - targets.decay[i]) / params.damping[i];
    }
    try {
        scheduler = std::make_unique<rdk::Scheduler>();  // 先建调度器：系统问题在连机器人之前就暴露。/ Create first so system issues surface before connecting.
        robot = std::make_unique<rdk::Robot>(params.robot_sn);
        for (int sig : {SIGINT, SIGTERM, SIGHUP}) {  // 在 RDK 初始化之后装，保证由这里接管信号。/ Installed after RDK init so these handlers win.
            std::signal(sig, OnSignal);
        }
        Prepare(*robot, params);
        const auto info = robot->info();
        const auto& arm1_K_nom = info.K_x_nom.at(rdk::JointGroup::ARM_1);
        const auto& arm1_q_min = info.q_min.at(rdk::JointGroup::ARM_1);  // --nullspace-follow 夹参考姿态用。/ For clamping the --nullspace-follow reference.
        const auto& arm1_q_max = info.q_max.at(rdk::JointGroup::ARM_1);

        need_stop = true;  // 从这里起结束时必须 Stop()。/ From here on, Stop() is required at exit.
        if (robot->mode() != rdk::Mode::IDLE) {
            robot->Stop();  // 机器人已静止，Stop() 只是切到 IDLE。/ The robot is still; Stop() only switches to IDLE.
        }
        // 导纳不用力控。以前的程序（降刚度浮动 --passive 等）可能开过被动力控，这里关掉，状态确定；文档：只能在 IDLE 下调用。
        // Admittance uses no force control. Earlier programs (reduced-stiffness floating with --passive, etc.) may have
        // enabled passive force control; turn it off for a known state. Per the docs this works only in IDLE.
        robot->SetPassiveForceControl(rdk::JointGroup::ARM_1, false);
        // ①② 用原语，只能在这个非实时模式下运行；这时还不能拖 ARM_1。拖动只在 ③，③ 全程是实时模式。
        // ①② use primitives, which run only in this non-RT mode; ARM_1 cannot be dragged yet. Dragging happens only in
        // ③, which stays in RT mode throughout.
        robot->SwitchMode(rdk::Mode::NRT_PRIMITIVE_EXECUTION);
        if (params.home) {
            std::cout << "① ARM_1 回正中（Home，关节速度 " << kHomeJntVelScale << "%），ARM_2 Hold。Ctrl+C 可随时停 ……"
                      << std::endl;
            RunArm1Primitive(*robot, rdk::PrimitiveArgs("Home", {{"jntVelScale", kHomeJntVelScale}}), kHomeTimeout,
                [&](const rdk::PrimitiveStates* state) {
                    if (!state) {
                        throw std::runtime_error("ARM_1 Home 没到位就结束了");
                    }
                    // 到位并停稳，才能接着清零。/ Reached and stopped before zeroing.
                    return rdk::utility::PrimitiveStateTrue(*state, "reachedTarget")
                           && robot->stopped().at(rdk::JointGroup::ARM_1);
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
                return !state || rdk::utility::PrimitiveStateTrue(*state, "terminated");  // 文档默认转移条件。/ Doc default transition condition.
            });
        PrintArm1Force(*robot, "后");

        // ③ 从这里到结束全程实时模式，不再切换。照官方 intermediate5/6 的顺序：先切模式，再设置，再读起始位姿。
        // ③ stays in RT mode from here to the end. Same order as the official intermediate5/6: switch mode, configure,
        // then read the start pose.
        robot->SetTimelinessFailureLimit(kTimelinessFailureLimit);
        robot->SwitchMode(rdk::Mode::RT_CARTESIAN_MOTION_FORCE);
        // 两条臂都是 6 轴纯位置控制、名义刚度、RDK 默认阻尼比 0.7，到结束都不改：ARM_1 的手感由导纳模型决定。
        // Both arms: pure 6-axis motion control, nominal stiffness, RDK default damping ratio 0.7, unchanged until the
        // end; ARM_1's feel comes from the admittance model.
        for (const auto group : {rdk::JointGroup::ARM_1, rdk::JointGroup::ARM_2}) {
            robot->SetForceControlAxis(group, {false, false, false, false, false, false});
            robot->SetCartesianImpedance(group, info.K_x_nom.at(group));
        }
        robot->SetNullSpaceObjectives(rdk::JointGroup::ARM_1, 0.0, 0.0, params.null_space_tracking);
        robot->SetNullSpaceObjectives(rdk::JointGroup::ARM_2, 0.0, 0.0, kArm2NullSpaceRefTracking);
        const auto states = robot->states();
        const auto& arm1 = states.at(rdk::JointGroup::ARM_1);
        const auto& arm2 = states.at(rdk::JointGroup::ARM_2);
        robot->SetNullSpacePosture(rdk::JointGroup::ARM_1, arm1.q);  // 肘部参考姿态 = 进入 ③ 时的姿态。/ Elbow reference = posture on entering ③.
        // 切模式时已自动设成当前姿态，这里写明，和 hold 位置一致。/ Already set to the current posture by the mode switch; stated explicitly to match the hold.
        robot->SetNullSpacePosture(rdk::JointGroup::ARM_2, arm2.q);
        targets.arm2_q = arm2.q;
        targets.arm1_start_pose = arm1.tcp_pose;
        // 只给位姿、目标速度 0，就是纯位置控制；ARM_1 的 pose_d、twist_d 导纳开始后由实时任务更新。
        // Pose only, zero target velocity: pure motion control. ARM_1's pose_d and twist_d are updated by the RT task
        // once admittance starts.
        targets.commands = {
            {rdk::JointGroup::ARM_1, rdk::RtCartesianCmd(arm1.tcp_pose)},
            {rdk::JointGroup::ARM_2, rdk::RtCartesianCmd(arm2.tcp_pose)},
        };
        std::array<double, rdk::kCartDoF> time_constant;  // 松手后速度衰减的时间常数 M/D [s]，启动时打印。/ Velocity decay time constant M/D [s], printed at start.
        for (size_t i = 0; i < rdk::kCartDoF; ++i) {
            time_constant[i] = params.mass[i] / params.damping[i];
        }
        std::cout << "ARM_1：名义刚度 K_x_nom [" << rdk::utility::Arr2Str(arm1_K_nom, 1)
                  << "] N/m、Nm/rad，阻尼比 0.7，全程不变；能拖的轴 " << AxisNames(false) << "：导纳 M ["
                  << rdk::utility::Arr2Str(params.mass, 3) << "] kg、kg·m²，D [" << rdk::utility::Arr2Str(params.damping, 2)
                  << "] N·s/m、N·m·s/rad，时间常数 M/D [" << rdk::utility::Arr2Str(time_constant, 3) << "] s，死区 "
                  << Num(kForceDeadband) << " N、" << Num(kMomentDeadband) << " N·m；锁定的轴 " << AxisNames(true)
                  << "：目标不动；跟不上导纳目标 " << Num(kMaxTrackPositionError * 1000.0) << " mm、"
                  << Num(kMaxTrackAngleError) << " rad 就停机；肘部拉回权重 " << Num(params.null_space_tracking)
                  << (params.null_space_follow ? "，参考姿态每 " + std::to_string(kMonitorPeriod.count())
                                                     + " ms 跟随实测关节位置"
                                               : std::string("，参考姿态 = 进入 ③ 时的姿态"))
                  << std::endl;
        // 打印 ARM_2 离位置极限最近的关节，排查 hold 时被推动的原因。/ Print ARM_2's joint nearest its limit, to diagnose hold being pushed.
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
            [&] { PeriodicTask(*robot, targets, rt_record); }, "HP periodic", 1, scheduler->max_priority());
        scheduler->Start();
        started = true;
        std::cout << "③ 实时模式 RT_CARTESIAN_MOTION_FORCE 已开始：ARM_2 hold 不动；ARM_1 先锁在当前位姿 ……" << std::endl;

        if (WaitWarmup(*robot) && !g_stop) {
            // 开始导纳：实时任务从下一次起按外力更新 ARM_1 的目标，从当前目标（起始位姿）、速度 0 开始积分；
            // 不改刚度、不切换控制模式，目标不会跳。
            // Start admittance: from the next cycle the RT task updates ARM_1's target from the wrench, integrating from
            // the current target (start pose) at zero velocity; no stiffness or mode change, so the target does not jump.
            g_admittance = true;
            admitting = true;
            std::cout << "ARM_1 导纳控制已开始，能拖的轴 " << AxisNames(false) << "，锁定的轴 " << AxisNames(true)
                      << "：推它就走，松手后减速停下、不弹回；ARM_2 hold 不动。按 Ctrl+C 结束。" << std::endl;
            if (log.is_open()) {
                WriteLogHeader(log);
                logging = true;
            }
        }
        const auto drag_start = std::chrono::steady_clock::now();
        while (!g_stop) {
            auto reason = NotRunningReason(*robot, rdk::Mode::RT_CARTESIAN_MOTION_FORCE);
            if (reason.empty()) {
                reason = HeartbeatLost();
            }
            if (!reason.empty()) {
                RequestStop(reason);
                break;
            }
            ++rt_mode_checks;  // 走到这里说明这次查到的控制模式仍是 RT_CARTESIAN_MOTION_FORCE。/ Reaching here means the mode is still RT_CARTESIAN_MOTION_FORCE.
            drag_seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - drag_start).count();
            const bool follow_null_space = admitting && params.null_space_follow;
            if (logging || follow_null_space) {
                const auto arm1_now = robot->states().at(rdk::JointGroup::ARM_1);
                if (logging) {  // 文件读写在主线程，不在实时任务里。/ File I/O stays in main, not in the RT task.
                    WriteLogRow(log, drag_seconds, arm1_now);
                }
                if (follow_null_space) {
                    // 肘部参考姿态 = 当前关节位置，夹在关节范围内（超出范围 SetNullSpacePosture 会报错）。
                    // 阻塞到送达，所以放在主线程，不在实时任务里；记下最长用时。
                    // Elbow reference = current joint positions, clamped to the joint range (SetNullSpacePosture
                    // rejects values outside it). It blocks until delivered, so it runs in main, not the RT task;
                    // the longest call is recorded.
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

    g_stop = true;  // 出错路径上也让实时任务停止发指令。/ Make the RT task stop sending on error paths too.
    for (int sig : {SIGINT, SIGTERM, SIGHUP}) {  // 停机过程中不再被信号打断。/ No signal interruptions during shutdown.
        std::signal(sig, SIG_IGN);
    }
    bool record_ready = false;  // 调度器正常停下后，rt_record 才不会再被实时任务改。/ rt_record is stable only after the scheduler stops cleanly.
    if (started) {
        try {
            scheduler->Stop();  // 先停实时任务，再整机 Stop()。/ Stop the RT task first, then Stop() the robot.
            record_ready = true;
        } catch (const std::exception& e) {
            std::cout << "调度器没有正常停止：" << e.what() << std::endl;
        }
    }
    if (need_stop) {
        try {
            robot->Stop();  // 整机停到 IDLE，两臂都保持当前位置。/ Whole robot to IDLE; both arms hold their current position.
            std::cout << "已 Stop()：机器人进入 IDLE 并保持当前位置。" << std::endl;
        } catch (const std::exception& e) {  // E-stop 按下时 Stop() 无法执行，机器人已由 E-stop 停住。/ With the E-stop pressed Stop() fails; the E-stop already stopped the robot.
            std::cout << "Stop() 没有执行：" << e.what() << std::endl;
        }
    }
    if (record_ready) {
        std::cout << "③ 期间 ARM_2 最大偏离 hold 位置：[" << rdk::utility::Arr2Str(rt_record.arm2_max_error, 4)
                  << "] rad；最大速度：[" << rdk::utility::Arr2Str(rt_record.arm2_max_vel, 3) << "] rad/s" << std::endl;
        if (admitting) {
            std::cout << "导纳期间 ARM_1 最大跟踪误差：位置 " << Num(rt_record.arm1_max_position_error * 1000.0)
                      << " mm、姿态 " << Num(rt_record.arm1_max_angle_error) << " rad（停机上限 "
                      << Num(kMaxTrackPositionError * 1000.0) << " mm、" << Num(kMaxTrackAngleError) << " rad）"
                      << std::endl;
        }
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
