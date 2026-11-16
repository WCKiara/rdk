// arm1_soft_stop.hpp 的离线单元测试（软停止指令第 9 节 Test A）：不连机器人、不用 RDK，零真机运动。
// 覆盖：二次轨迹边界、零速度和正负速度、锁住的轴、停止时间 T 的规则（速度大也照样执行、不设上限）、姿态（含超过半圈）、
// Stop Sign 锁存、不自动恢复透明、实测值无效或过期、时间异常、停稳超时、跟踪误差、刚度渐增公式、速度滤波、日志格式、
// 无锁队列，以及 1 kHz 闭环模拟（一般速度和大速度，实时抖动、丢周期、切运动控制的延迟、刚度渐增）中 ARM_2 指令逐字节不变。
// 闭环模拟的机械臂是很简单的质量-弹簧-阻尼模型，只用来检查状态机和轨迹的逻辑，不代表真机的跟踪效果。
//
// 编译运行（项目根目录；CMake 方式要装好 RDK，因为 script/CMakeLists.txt 开头就找 RDK）：
//   cmake -S script -B .build/script && cmake --build .build/script --target arm1_soft_stop_test
//   .build/script/arm1_soft_stop_test
// 没装 RDK 的电脑上直接用 g++：
//   g++ -std=c++20 -O2 -Wall -Wextra -o arm1_soft_stop_test script/arm1_soft_stop_test.cpp && ./arm1_soft_stop_test
// 加 --csv FILE.csv：把闭环模拟（不开刚度渐增）写成和 --log 同样格式的日志（关节列为 NaN），可以离线先试 plot_soft_stop.py。
// 全部通过退出码 0，有失败退出码 1。

#include "arm1_soft_stop.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iostream>
#include <limits>
#include <random>
#include <sstream>
#include <string>
#include <utility>

namespace {
using soft_stop::Arm1State;
using soft_stop::CartesianCmd;
using soft_stop::Measurement;
using soft_stop::Num;
using soft_stop::Pose;
using soft_stop::StopConfig;
using soft_stop::StopPlan;
using soft_stop::Vec6;

int g_checks = 0;
int g_failures = 0;

void Check(bool ok, const std::string& what, int line)
{
    ++g_checks;
    if (!ok) {
        ++g_failures;
        std::cout << "  失败（第 " << line << " 行）：" << what << "\n";
    }
}

void CheckNear(double a, double b, double tol, const std::string& what, int line)
{
    Check(std::abs(a - b) <= tol, what + "（" + Num(a) + " 对 " + Num(b) + "）", line);
}

#define CHECK(cond) Check((cond), #cond, __LINE__)
#define CHECK_NEAR(a, b, tol) CheckNear((a), (b), (tol), #a " ≈ " #b, __LINE__)

StopConfig DefaultConfig()  // 和 arm1_transparent_arm2_hold_rt_cartesian.cpp 的默认常量一样；改程序常量不影响这里。
{
    StopConfig c;
    c.free_axes = {true, true, true};
    c.max_linear_decel = 0.5;
    c.max_angular_decel = 1.0;
    c.min_time = 0.3;
    c.max_stale_ticks = 20;
    c.hold_linear_vel = 0.005;
    c.hold_angular_vel = 0.02;
    c.hold_position_tol = 0.005;
    c.hold_angle_tol = 0.03;
    c.hold_max_force = 20.0;
    c.hold_settle_time = 0.2;
    c.hold_timeout = 3.0;
    c.max_track_position_error = 0.05;
    c.max_track_angle_error = 0.3;
    return c;
}

Pose StartPose()  // 一个姿态不平凡的起点。
{
    const auto q = soft_stop::Exp({0.3, -0.2, 0.5});
    return {0.5, 0.1, 0.4, q[0], q[1], q[2], q[3]};
}

Measurement Measure(const Pose& pose, const Vec6& twist = {}, int stale_ticks = 0)
{
    Measurement m;
    m.tcp_pose = pose;
    m.tcp_twist = twist;
    m.stale_ticks = stale_ticks;
    return m;
}

CartesianCmd TransparentCmd(const Pose& pose)  // 原来的透明指令：位姿 + 目标力 0。
{
    CartesianCmd cmd;
    cmd.pose_d = pose;
    return cmd;
}

bool SameBytes(const CartesianCmd& a, const CartesianCmd& b)
{
    return std::memcmp(&a, &b, sizeof(CartesianCmd)) == 0;
}

bool Finite(const CartesianCmd& cmd)
{
    return soft_stop::AllFinite(cmd.pose_d) && soft_stop::AllFinite(cmd.wrench_d) && soft_stop::AllFinite(cmd.twist_d)
           && soft_stop::AllFinite(cmd.acc_d);
}

// 理想跟踪：实测正好等于目标。用来把状态机推到 HOLD。
Measurement Tracking(const CartesianCmd& cmd)
{
    return Measure(cmd.pose_d, cmd.twist_d);
}

void TestTrajectoryBoundaries()  // t = 0：p = p0、v = v0；t = T：p = p0 + v0 T/2、v = 0；中间按公式；a = -v0/T；导数一致。
{
    const auto c = DefaultConfig();
    const auto pose = StartPose();
    const Vec6 twist = {0.08, -0.05, 0.02, 0.1, -0.2, 0.15};
    StopPlan plan;
    CHECK(soft_stop::PlanStop(c, pose, Measure(pose, twist), 10.0, plan).empty());
    const double T = plan.T;
    CHECK_NEAR(T, 0.3, 1e-12);  // |v0| / a_max = 0.19 s、|ω0| / α_max = 0.27 s，都小于 T_min = 0.3 s。
    CHECK_NEAR(plan.t0, 10.0, 0.0);
    CartesianCmd cmd;
    soft_stop::SampleStop(plan, 0.0, cmd);
    for (size_t i = 0; i < 3; ++i) {
        CHECK_NEAR(cmd.pose_d[i], pose[i], 1e-12);
        CHECK_NEAR(cmd.twist_d[i], twist[i], 1e-12);
        CHECK_NEAR(cmd.acc_d[i], -twist[i] / T, 1e-12);
        CHECK_NEAR(cmd.twist_d[3 + i], twist[3 + i], 1e-12);
        CHECK_NEAR(cmd.acc_d[3 + i], -twist[3 + i] / T, 1e-12);
    }
    CHECK_NEAR(soft_stop::AngleBetween(soft_stop::QuatOf(cmd.pose_d), soft_stop::QuatOf(pose)), 0.0, 1e-9);
    for (double w : cmd.wrench_d) {
        CHECK(w == 0.0);
    }
    soft_stop::SampleStop(plan, T, cmd);
    for (size_t i = 0; i < 3; ++i) {
        CHECK_NEAR(cmd.pose_d[i], pose[i] + 0.5 * twist[i] * T, 1e-12);
    }
    for (size_t i = 0; i < 6; ++i) {
        CHECK(cmd.twist_d[i] == 0.0);
        CHECK(cmd.acc_d[i] == 0.0);
    }
    for (double t : {0.05, 0.1, 0.15, 0.2, 0.29}) {  // 中间点：p = p0 + v0 (t - t²/2T)，v = v0 (1 - t/T)。
        soft_stop::SampleStop(plan, t, cmd);
        for (size_t i = 0; i < 3; ++i) {
            CHECK_NEAR(cmd.pose_d[i], pose[i] + twist[i] * (t - t * t / (2.0 * T)), 1e-12);
            CHECK_NEAR(cmd.twist_d[i], twist[i] * (1.0 - t / T), 1e-12);
            CHECK_NEAR(cmd.acc_d[i], -twist[i] / T, 1e-12);
        }
    }
    const double h = 1e-6;  // 数值导数：位置的导数 = 目标速度，速度的导数 = 目标加速度。
    for (double t : {0.01, 0.1, 0.2, 0.28}) {
        CartesianCmd before, mid, after;
        soft_stop::SampleStop(plan, t - h, before);
        soft_stop::SampleStop(plan, t, mid);
        soft_stop::SampleStop(plan, t + h, after);
        for (size_t i = 0; i < 3; ++i) {
            CHECK_NEAR((after.pose_d[i] - before.pose_d[i]) / (2.0 * h), mid.twist_d[i], 1e-6);
            CHECK_NEAR((after.twist_d[i] - before.twist_d[i]) / (2.0 * h), mid.acc_d[i], 1e-6);
        }
    }
    CHECK_NEAR(plan.distance, 0.5 * soft_stop::Norm3(0.08, -0.05, 0.02) * T, 1e-12);
    CHECK_NEAR(soft_stop::Distance(plan.stop_pose, pose), plan.distance, 1e-12);
}

void TestHoldAfterT()  // t > T：终点不变，速度、加速度为 0，逐字节恒定。
{
    const auto pose = StartPose();
    StopPlan plan;
    CHECK(soft_stop::PlanStop(DefaultConfig(), pose, Measure(pose, {0.1, 0.0, -0.05, 0.0, 0.3, 0.0}), 0.0, plan).empty());
    CartesianCmd at_T, later;
    soft_stop::SampleStop(plan, plan.T, at_T);
    for (double t : {plan.T + 1e-6, plan.T + 0.001, plan.T + 1.0, plan.T + 1000.0}) {
        soft_stop::SampleStop(plan, t, later);
        CHECK(SameBytes(at_T, later));
    }
    CHECK(at_T.pose_d == plan.stop_pose);
}

void TestZeroVelocity()  // 速度为 0：T = T_min，任何时刻都停在原地。
{
    const auto c = DefaultConfig();
    const auto pose = StartPose();
    StopPlan plan;
    CHECK(soft_stop::PlanStop(c, pose, Measure(pose), 0.0, plan).empty());
    CHECK_NEAR(plan.T, c.min_time, 0.0);
    CHECK(plan.distance == 0.0 && plan.angle == 0.0);
    for (double t : {0.0, 0.1, 0.3, 5.0}) {
        CartesianCmd cmd;
        soft_stop::SampleStop(plan, t, cmd);
        for (size_t i = 0; i < 3; ++i) {
            CHECK(cmd.pose_d[i] == pose[i]);
        }
        CHECK_NEAR(soft_stop::AngleBetween(soft_stop::QuatOf(cmd.pose_d), soft_stop::QuatOf(pose)), 0.0, 1e-9);
        for (size_t i = 0; i < 6; ++i) {
            CHECK(cmd.twist_d[i] == 0.0 && cmd.acc_d[i] == 0.0);
        }
    }
}

void TestDirections()  // 正、负速度都停在 p0 ± |v0| T/2，减速过程中目标速度不反向。
{
    const auto pose = StartPose();
    const Vec6 twist = {0.1, -0.1, 0.0, 0.0, 0.0, 0.0};
    StopPlan plan;
    CHECK(soft_stop::PlanStop(DefaultConfig(), pose, Measure(pose, twist), 0.0, plan).empty());
    CHECK_NEAR(plan.stop_pose[0], pose[0] + 0.5 * 0.1 * plan.T, 1e-12);
    CHECK_NEAR(plan.stop_pose[1], pose[1] - 0.5 * 0.1 * plan.T, 1e-12);
    CHECK_NEAR(plan.stop_pose[2], pose[2], 0.0);
    double last_speed = 1e9;
    for (int k = 0; k <= 300; ++k) {
        CartesianCmd cmd;
        soft_stop::SampleStop(plan, plan.T * k / 300.0, cmd);
        CHECK(cmd.twist_d[0] >= 0.0 && cmd.twist_d[1] <= 0.0);
        const double speed = soft_stop::LinearNorm(cmd.twist_d);
        CHECK(speed <= last_speed + 1e-15);  // 目标速度单调降到 0。
        last_speed = speed;
    }
    CHECK(last_speed == 0.0);
}

void TestLockedAxes()  // 锁住的轴（透明时不是力控）：保持原来的目标、不减速，测到的速度不算进 T。
{
    auto c = DefaultConfig();
    c.free_axes = {true, false, true};
    const auto pose = StartPose();
    auto transparent_pose = pose;
    transparent_pose[1] = pose[1] + 0.002;  // 锁住的 Y 有 2 mm 跟踪误差。
    StopPlan plan;
    CHECK(soft_stop::PlanStop(c, transparent_pose, Measure(pose, {0.05, 0.3, 0.0, 0.0, 0.0, 0.0}), 0.0, plan).empty());
    CHECK(plan.p0[1] == transparent_pose[1] && plan.v0[1] == 0.0);
    CHECK_NEAR(plan.T, c.min_time, 0.0);  // Y 的 0.3 m/s 不算：只按 X 的 0.05 m/s。
    for (double t : {0.0, 0.1, 0.3, 1.0}) {
        CartesianCmd cmd;
        soft_stop::SampleStop(plan, t, cmd);
        CHECK(cmd.pose_d[1] == transparent_pose[1] && cmd.twist_d[1] == 0.0 && cmd.acc_d[1] == 0.0);
    }
}

void TestStopTimeRule()  // T = max(T_min, |v0|/a_max, |ω0|/α_max)；速度再大也照样执行，T、停止距离跟着变大，没有上限。
{
    const auto c = DefaultConfig();
    const auto pose = StartPose();
    StopPlan plan;
    CHECK(soft_stop::PlanStop(c, pose, Measure(pose, {0.2, 0.0, 0.0, 0.0, 0.0, 0.0}), 0.0, plan).empty());
    CHECK_NEAR(plan.T, 0.4, 1e-12);
    CHECK_NEAR(plan.distance, 0.04, 1e-12);
    // 速度大：照样规划，停止距离 = |v0|²/(2 a_max)，目标减速度正好是 a_max，不超过它。
    for (double v : {0.25, 0.5, 1.0, 2.0}) {
        CHECK(soft_stop::PlanStop(c, pose, Measure(pose, {0.0, v, 0.0, 0.0, 0.0, 0.0}), 0.0, plan).empty());
        CHECK_NEAR(plan.T, v / c.max_linear_decel, 1e-12);
        CHECK_NEAR(plan.distance, v * v / (2.0 * c.max_linear_decel), 1e-12);
        CHECK_NEAR(soft_stop::Distance(plan.stop_pose, pose), plan.distance, 1e-9);
        CartesianCmd cmd;
        soft_stop::SampleStop(plan, 0.0, cmd);
        CHECK_NEAR(soft_stop::LinearNorm(cmd.acc_d), c.max_linear_decel, 1e-12);
    }
    std::cout << "  大速度也执行：1 m/s 时 T = 2 s、停止距离 0.5 m；2 m/s 时 T = 4 s、停止距离 2 m\n";
    soft_stop::Arm1StopMachine fast(c, TransparentCmd(pose));  // 状态机也照样从 TRANSPARENT 进 SOFT_STOP。
    CartesianCmd cmd;
    CHECK(fast.Step(0.0, true, Measure(pose, {0.6, -0.3, 0.2, 0.0, 0.0, 2.0}), cmd).empty());
    CHECK(fast.state() == Arm1State::SOFT_STOP);
    // 转动：|ω0| = 0.5 → T = 0.5 s、转 0.125 rad；|ω0| = 3 rad/s → T = 3 s、转 4.5 rad，超过半圈也照样执行。
    CHECK(soft_stop::PlanStop(c, pose, Measure(pose, {0.0, 0.0, 0.0, 0.5, 0.0, 0.0}), 0.0, plan).empty());
    CHECK_NEAR(plan.T, 0.5, 1e-12);
    CHECK_NEAR(plan.angle, 0.125, 1e-12);
    CHECK(soft_stop::PlanStop(c, pose, Measure(pose, {0.0, 0.0, 0.0, 0.0, 3.0, 0.0}), 0.0, plan).empty());
    CHECK_NEAR(plan.T, 3.0, 1e-12);
    CHECK_NEAR(plan.angle, 4.5, 1e-12);
    // 平移、转动一起：取较长的那个。
    CHECK(soft_stop::PlanStop(c, pose, Measure(pose, {0.2, 0.0, 0.0, 0.0, 0.0, 0.5}), 0.0, plan).empty());
    CHECK_NEAR(plan.T, 0.5, 1e-12);
    CHECK_NEAR(plan.distance, 0.05, 1e-12);
    // 指令文档的例子：T = 1 s 时 0.01 m/s 走 5 mm，0.10 m/s 走 50 mm。
    auto one_second = c;
    one_second.min_time = 1.0;
    CHECK(soft_stop::PlanStop(one_second, pose, Measure(pose, {0.01, 0.0, 0.0, 0.0, 0.0, 0.0}), 0.0, plan).empty());
    CHECK_NEAR(plan.distance, 0.005, 1e-12);
    CHECK(soft_stop::PlanStop(one_second, pose, Measure(pose, {0.10, 0.0, 0.0, 0.0, 0.0, 0.0}), 0.0, plan).empty());
    CHECK_NEAR(plan.distance, 0.05, 1e-12);
}

// 姿态终点 = Exp(ω0 T/2) R0；单位四元数；符号连续；数值角速度 = ω_d。|ω0| ≈ 2.9 rad/s 时要转 4.3 rad，超过半圈也一样。
void TestOrientation()
{
    const auto pose = StartPose();
    const auto q0 = soft_stop::QuatOf(pose);
    for (const Vec6& twist : {Vec6 {0.0, 0.0, 0.0, 0.1, -0.2, 0.15}, Vec6 {0.0, 0.0, 0.0, 1.2, 2.5, -1.0}}) {
        StopPlan plan;
        CHECK(soft_stop::PlanStop(DefaultConfig(), pose, Measure(pose, twist), 0.0, plan).empty());
        const double T = plan.T;
        const auto expected
            = soft_stop::Multiply(soft_stop::Exp({twist[3] * T / 2.0, twist[4] * T / 2.0, twist[5] * T / 2.0}), q0);
        CHECK_NEAR(soft_stop::AngleBetween(soft_stop::QuatOf(plan.stop_pose), expected), 0.0, 1e-9);
        if (plan.angle < 3.0) {  // AngleBetween 只到 π。
            CHECK_NEAR(soft_stop::AngleBetween(q0, soft_stop::QuatOf(plan.stop_pose)), plan.angle, 1e-9);
        }
        soft_stop::Quat last = q0;
        for (int k = 0; k <= 400; ++k) {
            CartesianCmd cmd;
            soft_stop::SampleStop(plan, 1.2 * T * k / 400.0, cmd);
            const auto q = soft_stop::QuatOf(cmd.pose_d);
            CHECK_NEAR(soft_stop::QuatNorm(q), 1.0, 1e-12);
            CHECK(q[0] * last[0] + q[1] * last[1] + q[2] * last[2] + q[3] * last[3] > 0.0);  // 不突然换号。
            last = q;
        }
        const double h = 1e-6;
        for (double s : {0.03, 0.3, 0.6, 0.93}) {
            const double t = s * T;
            CartesianCmd before, mid, after;
            soft_stop::SampleStop(plan, t - h, before);
            soft_stop::SampleStop(plan, t, mid);
            soft_stop::SampleStop(plan, t + h, after);
            // 世界系角速度：q(t+h) ⊗ q(t-h)⁻¹ 的旋转向量 / 2h。
            const auto r = soft_stop::Log(soft_stop::Multiply(
                soft_stop::QuatOf(after.pose_d), soft_stop::Conjugate(soft_stop::QuatOf(before.pose_d))));
            for (size_t i = 0; i < 3; ++i) {
                CHECK_NEAR(r[i] / (2.0 * h), mid.twist_d[3 + i], 1e-6);
                CHECK_NEAR(mid.acc_d[3 + i], -twist[3 + i] / T, 1e-12);
            }
        }
    }
}

// 用理想跟踪把状态机从 now 推进到 until，返回最后一个错误（空 = 一直正常）。
std::string Advance(soft_stop::Arm1StopMachine& machine, double& now, double until, bool stop_requested, CartesianCmd& cmd)
{
    for (; now < until; now += 0.001) {
        if (auto error = machine.Step(now, stop_requested, Tracking(cmd), cmd); !error.empty()) {
            return error;
        }
    }
    return "";
}

void TestLatchOnce()  // Stop Sign 连续来 100 次（测到的速度也在变）：只从 TRANSPARENT 跳一次，t0、v0、T 不变。
{
    const auto c = DefaultConfig();
    const auto pose = StartPose();
    const Vec6 twist = {0.08, -0.05, 0.02, 0.1, -0.2, 0.15};
    const auto transparent = TransparentCmd(pose);
    soft_stop::Arm1StopMachine machine(c, transparent);
    CartesianCmd cmd;
    for (int k = 0; k < 10; ++k) {  // 没有 Stop Sign：原样发原来的透明指令。
        CHECK(machine.Step(k * 0.001, false, Measure(pose, twist), cmd).empty());
        CHECK(SameBytes(cmd, transparent));
    }
    CHECK(machine.state() == Arm1State::TRANSPARENT);
    const double t0 = 0.01;
    int latches = 0;
    StopPlan first;
    for (int k = 0; k <= 100; ++k) {
        const double now = t0 + k * 0.001;
        Measurement m = Measure(pose, twist);
        if (k > 0) {
            CartesianCmd expected;
            soft_stop::SampleStop(first, now - t0, expected);
            m = Measure(expected.pose_d, {0.2, 0.1, -0.1, 0.5, 0.0, 0.0});  // 位置跟得上，速度读数变了。
        }
        const auto before = machine.state();
        CHECK(machine.Step(now, true, m, cmd).empty());
        if (before == Arm1State::TRANSPARENT && machine.state() != Arm1State::TRANSPARENT) {
            ++latches;
            first = machine.plan();
        }
    }
    CHECK(latches == 1);
    CHECK(machine.plan().t0 == t0);
    CHECK(machine.plan().v0 == first.v0 && machine.plan().w0 == first.w0 && machine.plan().T == first.T);
}

void TestNoAutoResume()  // HOLD 之后不管有没有 Stop Sign，都不会自己回到 TRANSPARENT。
{
    const auto c = DefaultConfig();
    const auto pose = StartPose();
    soft_stop::Arm1StopMachine machine(c, TransparentCmd(pose));
    CartesianCmd cmd;
    CHECK(machine.Step(0.0, true, Measure(pose, {0.05, 0.0, 0.0, 0.0, 0.0, 0.1}), cmd).empty());
    double now = 0.001;
    CHECK(Advance(machine, now, 1.0, true, cmd).empty());
    CHECK(machine.state() == Arm1State::HOLD);
    CHECK_NEAR(machine.hold_time(), c.min_time + c.hold_settle_time, 0.003);
    CHECK(Advance(machine, now, 3.0, false, cmd).empty());
    CHECK(machine.state() == Arm1State::HOLD);
    CHECK(cmd.pose_d == machine.plan().stop_pose);
}

void TestBadMeasurement()  // 实测值无效或过期：不开始软停止，仍是 TRANSPARENT，返回原因。速度大不算无效。
{
    const auto c = DefaultConfig();
    const auto pose = StartPose();
    CartesianCmd cmd;
    auto expect_refused = [&](const Measurement& m, const char* what) {
        soft_stop::Arm1StopMachine machine(c, TransparentCmd(pose));
        const auto error = machine.Step(0.0, true, m, cmd);
        Check(!error.empty() && machine.state() == Arm1State::TRANSPARENT, what, __LINE__);
    };
    auto nan_pose = pose;
    nan_pose[0] = soft_stop::kNaN;
    expect_refused(Measure(nan_pose), "位置 NaN");
    expect_refused(Measure(pose, {0.0, soft_stop::kNaN, 0.0, 0.0, 0.0, 0.0}), "速度 NaN");
    auto bad_quat = pose;
    bad_quat[3] *= 1.2;
    expect_refused(Measure(bad_quat), "四元数不是单位长度");
    expect_refused(Measure(pose, {}, c.max_stale_ticks + 1), "状态过期");
    expect_refused(Measure(pose, {std::numeric_limits<double>::infinity(), 0.0, 0.0, 0.0, 0.0, 0.0}), "速度无穷大");
    soft_stop::Arm1StopMachine boundary(c, TransparentCmd(pose));
    CHECK(boundary.Step(0.0, true, Measure(pose, {}, c.max_stale_ticks), cmd).empty());  // 正好等于上限还算新鲜。
    // SOFT_STOP 中实测值变成 NaN 或过期：报错，不继续。
    for (int kind = 0; kind < 2; ++kind) {
        soft_stop::Arm1StopMachine machine(c, TransparentCmd(pose));
        CHECK(machine.Step(0.0, true, Measure(pose, {0.05, 0.0, 0.0, 0.0, 0.0, 0.0}), cmd).empty());
        auto m = Tracking(cmd);
        if (kind == 0) {
            m.tcp_twist[4] = soft_stop::kNaN;
        } else {
            m.stale_ticks = c.max_stale_ticks + 1;
        }
        CHECK(!machine.Step(0.001, true, m, cmd).empty());
    }
}

void TestBadTime()  // 时间 NaN 报错；时间倒退、大段丢周期时指令仍有限、仍按公式。
{
    const auto c = DefaultConfig();
    const auto pose = StartPose();
    CartesianCmd cmd;
    soft_stop::Arm1StopMachine nan_time(c, TransparentCmd(pose));
    CHECK(!nan_time.Step(soft_stop::kNaN, false, Measure(pose), cmd).empty());
    soft_stop::Arm1StopMachine machine(c, TransparentCmd(pose));
    const Vec6 twist = {0.08, 0.0, 0.0, 0.0, 0.0, 0.2};
    CHECK(machine.Step(5.0, true, Measure(pose, twist), cmd).empty());
    const auto plan = machine.plan();
    CHECK(machine.Step(4.995, true, Tracking(cmd), cmd).empty());  // 时间倒退 5 ms：按 t = 0。
    CHECK(Finite(cmd) && cmd.pose_d[0] == pose[0]);
    for (double t : {0.1, 0.25, 0.26, 2.0}) {  // 每次跳很多周期。
        const double now = 5.0 + t;
        CartesianCmd expected;
        soft_stop::SampleStop(plan, now - plan.t0, expected);  // 和状态机一样用 now - t0，舍入一致才能逐字节比。
        CHECK(machine.Step(now, true, Measure(expected.pose_d, expected.twist_d), cmd).empty());
        CHECK(Finite(cmd) && SameBytes(cmd, expected));
    }
}

void TestSettleTimeout()  // t ≥ T 后一直没停稳：到 T + hold_timeout 报错，之前不报。
{
    const auto c = DefaultConfig();
    const auto pose = StartPose();
    soft_stop::Arm1StopMachine machine(c, TransparentCmd(pose));
    CartesianCmd cmd;
    CHECK(machine.Step(0.0, true, Measure(pose, {0.05, 0.0, 0.0, 0.0, 0.0, 0.0}), cmd).empty());
    std::string error;
    double failed_at = soft_stop::kNaN;
    for (int k = 1; k < 6000 && error.empty(); ++k) {
        auto m = Tracking(cmd);
        m.tcp_twist[0] = 0.01;  // 一直有 10 mm/s 的速度。
        error = machine.Step(k * 0.001, true, m, cmd);
        failed_at = k * 0.001;
    }
    CHECK(error.find("没停稳") != std::string::npos);
    CHECK(failed_at > c.min_time + c.hold_timeout && failed_at < c.min_time + c.hold_timeout + 0.003);
    CHECK(machine.state() == Arm1State::SOFT_STOP);
    std::cout << "  超时示例：" << error << "\n";
}

void TestTrackError()  // SOFT_STOP 中实测离目标超过跟踪误差上限：马上报错。
{
    const auto c = DefaultConfig();
    const auto pose = StartPose();
    for (double offset : {0.04, 0.06}) {
        soft_stop::Arm1StopMachine machine(c, TransparentCmd(pose));
        CartesianCmd cmd;
        CHECK(machine.Step(0.0, true, Measure(pose, {0.05, 0.0, 0.0, 0.0, 0.0, 0.0}), cmd).empty());
        auto m = Tracking(cmd);
        m.tcp_pose[2] += offset;
        const auto error = machine.Step(0.001, true, m, cmd);
        CHECK(error.empty() == (offset < c.max_track_position_error));
    }
    soft_stop::Arm1StopMachine rotated(c, TransparentCmd(pose));
    CartesianCmd cmd;
    CHECK(rotated.Step(0.0, true, Measure(pose), cmd).empty());
    auto m = Tracking(cmd);
    const auto q = soft_stop::Multiply(soft_stop::Exp({0.0, 0.35, 0.0}), soft_stop::QuatOf(cmd.pose_d));
    std::copy(q.begin(), q.end(), m.tcp_pose.begin() + 3);
    CHECK(!rotated.Step(0.001, true, m, cmd).empty());
}

void TestStiffness()  // K(t) = K0 + (Kf - K0)(t/T)²；ramp 关时一步到 Kf。
{
    const Vec6 k0 = {0.0, 0.0, 1000.0, 0.0, 5.0, 0.0};
    const Vec6 kf = {4000.0, 4000.0, 4000.0, 300.0, 300.0, 300.0};
    const double T = 0.4;
    const auto at0 = soft_stop::StiffnessAt(k0, kf, 0.0, T);
    const auto atT = soft_stop::StiffnessAt(k0, kf, T, T);
    const auto half = soft_stop::StiffnessAt(k0, kf, T / 2.0, T);
    for (size_t i = 0; i < 6; ++i) {
        CHECK(at0[i] == k0[i]);
        CHECK(atT[i] == kf[i]);
        CHECK_NEAR(half[i], k0[i] + (kf[i] - k0[i]) / 4.0, 1e-9);
        CHECK(soft_stop::StiffnessAt(k0[i], kf[i], -1.0, T) == k0[i]);
        CHECK(soft_stop::StiffnessAt(k0[i], kf[i], 10.0, T) == kf[i]);
        CHECK(soft_stop::StopStiffness(false, k0, kf, 0.0, T)[i] == kf[i]);
    }
    double last = -1.0;
    for (int k = 0; k <= 100; ++k) {  // 单调不减。
        const double v = soft_stop::StiffnessAt(0.0, 4000.0, T * k / 100.0, T);
        CHECK(v >= last);
        last = v;
    }
}

void TestTwistFilter()  // 滑动平均：样本不满时按已有的平均；满了只看最近 N 个。
{
    soft_stop::TwistFilter filter(5);
    const double expected[] = {1.0, 1.5, 2.0, 2.5, 3.0, 4.0, 5.0};
    for (int k = 1; k <= 7; ++k) {
        const auto& avg = filter.Update({double(k), 0.0, 0.0, 0.0, 0.0, -double(k)});
        CHECK_NEAR(avg[0], expected[k - 1], 1e-12);
        CHECK_NEAR(avg[5], -expected[k - 1], 1e-12);
    }
    soft_stop::TwistFilter one(0);  // 少于 1 个按 1 个：不滤波。
    CHECK(one.Update({3.0, 0.0, 0.0, 0.0, 0.0, 0.0})[0] == 3.0);
    CHECK(one.Update({7.0, 0.0, 0.0, 0.0, 0.0, 0.0})[0] == 7.0);
}

void TestStaleCounter()
{
    soft_stop::StaleCounter stale;
    CHECK(stale.Update(10, 100) == 0);
    CHECK(stale.Update(10, 100) == 1);
    CHECK(stale.Update(10, 100) == 2);
    CHECK(stale.Update(10, 1100) == 0);
    CHECK(stale.Update(11, 1100) == 0);
}

void TestLogFormat()  // 表头和数据行的列数一样，NaN 写成 nan。
{
    std::ostringstream header, row;
    soft_stop::WriteLogHeader(header);
    soft_stop::WriteLogRow(row, soft_stop::LogRow{});
    const auto count = [](const std::string& s) { return std::count(s.begin(), s.end(), ',') + 1; };
    CHECK(count(header.str()) == count(row.str()));
    CHECK(count(header.str()) == 91);
    CHECK(header.str().rfind("time_s,rt_cycle_us,stop_sign,arm1_state", 0) == 0);
    CHECK(row.str().rfind("nan,nan", 0) == 0);
}

void TestRing()  // 无锁队列：先进先出；满了 TryPush 返回 false；绕圈正常。
{
    soft_stop::SpscRing<int> ring(3);
    CHECK(ring.TryPush(1) && ring.TryPush(2) && ring.TryPush(3));
    CHECK(!ring.TryPush(4));
    int v = 0;
    CHECK(ring.TryPop(v) && v == 1);
    CHECK(ring.TryPush(4));
    CHECK(ring.TryPop(v) && v == 2);
    CHECK(ring.TryPop(v) && v == 3);
    CHECK(ring.TryPop(v) && v == 4);
    CHECK(!ring.TryPop(v));
    bool order_ok = true;
    for (int k = 0; k < 1000; ++k) {
        order_ok = order_ok && ring.TryPush(k) && ring.TryPush(k + 1) && ring.TryPop(v) && v == k && ring.TryPop(v)
                   && v == k + 1;
    }
    CHECK(order_ok);
}

void TestConfigCheck()
{
    CHECK(soft_stop::CheckConfig(DefaultConfig()).empty());
    auto c = DefaultConfig();
    c.min_time = 0.0;
    CHECK(!soft_stop::CheckConfig(c).empty());
    c = DefaultConfig();
    c.max_linear_decel = -1.0;
    CHECK(!soft_stop::CheckConfig(c).empty());
    c = DefaultConfig();
    c.hold_position_tol = c.max_track_position_error;
    CHECK(!soft_stop::CheckConfig(c).empty());
    c = DefaultConfig();
    c.max_stale_ticks = 0;
    CHECK(!soft_stop::CheckConfig(c).empty());
}

// ================================ 1 kHz 闭环模拟 ================================
constexpr double kSimMass = 8.0;     // 每个平移轴的等效质量 [kg]，只用于模拟。
constexpr double kSimInertia = 0.2;  // 每个转动轴的等效惯量 [kg·m²]
constexpr double kSimZeta = 0.7;     // 阻尼比，同 RDK 默认值。

struct SimArm  // 很简单的 ARM_1 末端：运动控制时按刚度 K、阻尼比 0.7 跟踪目标并前馈目标加速度；还没切到运动控制时不受力，
               // 按当前速度滑行（透明、没人推）。
{
    soft_stop::Vec3 p = {};
    soft_stop::Vec3 v = {};
    soft_stop::Quat q = {1.0, 0.0, 0.0, 0.0};
    soft_stop::Vec3 w = {};

    void Advance(double dt, const CartesianCmd& cmd, bool motion, const Vec6& K)
    {
        constexpr int kSubsteps = 8;  // 子步积分：最长 6 ms 的周期间隔也稳定。
        const double h = dt / kSubsteps;
        for (int s = 0; s < kSubsteps; ++s) {
            const auto rot_error
                = soft_stop::Log(soft_stop::Multiply(soft_stop::QuatOf(cmd.pose_d), soft_stop::Conjugate(q)));
            for (size_t i = 0; i < 3; ++i) {
                double a = 0.0;
                double alpha = 0.0;
                if (motion) {
                    a = (K[i] * (cmd.pose_d[i] - p[i]) + 2.0 * kSimZeta * std::sqrt(K[i] * kSimMass) * (cmd.twist_d[i] - v[i]))
                            / kSimMass
                        + cmd.acc_d[i];
                    alpha = (K[3 + i] * rot_error[i]
                                + 2.0 * kSimZeta * std::sqrt(K[3 + i] * kSimInertia) * (cmd.twist_d[3 + i] - w[i]))
                                / kSimInertia
                            + cmd.acc_d[3 + i];
                }
                v[i] += a * h;
                p[i] += v[i] * h;
                w[i] += alpha * h;
            }
            q = soft_stop::Normalized(soft_stop::Multiply(soft_stop::Exp({w[0] * h, w[1] * h, w[2] * h}), q));
        }
    }

    Pose pose() const { return {p[0], p[1], p[2], q[0], q[1], q[2], q[3]}; }
    Vec6 twist() const { return {v[0], v[1], v[2], w[0], w[1], w[2]}; }
};

struct SimCommands  // 每个周期一起发的两条臂指令：arm2 构造后不再改，状态机只拿到 arm1。
{
    CartesianCmd arm1;
    CartesianCmd arm2;
};

// twist0：收到 Stop Sign 之前 ARM_1 滑行的速度。
void TestClosedLoopSim(const char* label, bool ramp, const Vec6& twist0, std::ostream* csv)
{
    const auto c = DefaultConfig();
    const Vec6 K_nom = {4000.0, 4000.0, 4000.0, 300.0, 300.0, 300.0};  // 假设的名义刚度，只用于模拟。
    const Vec6 K0 = {};  // 透明时 X Y Z 力控、转动刚度 0：实际刚度都按 0。
    const Vec6 K_stop = K_nom;
    constexpr double kStopAt = 0.5;           // 开始后 0.5 s 收到 Stop Sign，之后又连续来 100 次。
    constexpr double kReconfigureDelay = 0.03;  // 主线程切运动控制（两个阻塞调用）的延迟。
    constexpr double kRampPeriod = 0.05;      // 刚度渐增时设刚度的间隔。
    SimArm arm;
    arm.p = {0.5, 0.1, 0.4};
    arm.v = {twist0[0], twist0[1], twist0[2]};
    arm.w = {twist0[3], twist0[4], twist0[5]};
    arm.q = soft_stop::Exp({0.3, -0.2, 0.5});
    soft_stop::Arm1StopMachine machine(c, TransparentCmd(arm.pose()));
    SimCommands commands;
    commands.arm1 = TransparentCmd(arm.pose());
    commands.arm2.pose_d = {0.3, -0.4, 0.5, 1.0, 0.0, 0.0, 0.0};  // ARM_2 hold 位姿：wrench、twist、acc 都是 0。
    const CartesianCmd arm2_initial = commands.arm2;
    soft_stop::TwistFilter filter(5);
    soft_stop::StaleCounter stale;
    std::mt19937 rng(20261115);
    std::uniform_real_distribution<double> jitter(-0.0002, 0.0003);
    const double start = 100.0;
    double now = start;
    uint64_t stop_signs = 0;
    bool motion = false;
    bool ramp_done = false;
    Vec6 K_set = K0;
    double next_ramp = 0.0;
    int latches = 0, ticks = 0, ramp_calls = 0;
    bool arm2_constant = true, finite = true;
    std::string error;
    double max_track = 0.0;
    Pose latch_pose = {};
    Pose hold_pose = {};
    if (csv) {
        soft_stop::WriteLogHeader(*csv);
    }
    while (now - start < 4.0 && error.empty()) {
        ++ticks;
        const double dt = ticks % 400 == 0 ? 0.006 : 0.001 + jitter(rng);  // 1 ms ± 抖动，每 400 个周期丢 5 个。
        arm.Advance(dt, commands.arm1, motion, K_set);
        now += dt;
        // 主线程：Stop Sign；锁存后切运动控制、刚度渐增。
        if (now - start >= kStopAt && stop_signs <= 100) {
            ++stop_signs;
        }
        if (machine.state() != Arm1State::TRANSPARENT && !ramp_done) {
            const double t = now - machine.plan().t0;
            if (!motion && t >= kReconfigureDelay) {
                K_set = soft_stop::StopStiffness(ramp, K0, K_stop, t, machine.plan().T);
                motion = true;
                ramp_done = !ramp;
                next_ramp = t + kRampPeriod;
                ++ramp_calls;
            } else if (motion && t >= next_ramp) {
                K_set = soft_stop::StiffnessAt(K0, K_stop, t, machine.plan().T);
                ramp_done = t >= machine.plan().T;
                next_ramp = t + kRampPeriod;
                ++ramp_calls;
            }
        }
        // 实时任务。
        Measurement m;
        m.tcp_pose = arm.pose();
        m.tcp_twist = filter.Update(arm.twist());
        m.stale_ticks = stale.Update(static_cast<int>(now), static_cast<int>(std::fmod(now, 1.0) * 1e9));
        const auto before = machine.state();
        error = machine.Step(now, stop_signs > 0, m, commands.arm1);
        if (before == Arm1State::TRANSPARENT && machine.state() != Arm1State::TRANSPARENT) {
            ++latches;
            latch_pose = m.tcp_pose;
        }
        if (before == Arm1State::SOFT_STOP && machine.state() == Arm1State::HOLD) {
            hold_pose = m.tcp_pose;
        }
        arm2_constant = arm2_constant && SameBytes(commands.arm2, arm2_initial);
        finite = finite && Finite(commands.arm1);
        if (machine.state() != Arm1State::TRANSPARENT) {
            max_track = std::max(max_track, soft_stop::Distance(m.tcp_pose, commands.arm1.pose_d));
        }
        if (csv) {
            soft_stop::LogRow row;
            row.time_s = now - start;
            row.rt_cycle_us = dt * 1e6;
            row.stop_sign = static_cast<double>(stop_signs);
            row.arm1_state = static_cast<double>(machine.state());
            if (machine.state() != Arm1State::TRANSPARENT) {
                row.stop_elapsed_s = now - machine.plan().t0;
                row.stop_T_s = machine.plan().T;
            }
            row.arm1_pose = m.tcp_pose;
            row.arm1_twist = arm.twist();
            row.arm1_force = {0.0, 0.0, 0.0};
            row.target_pose = commands.arm1.pose_d;
            row.target_twist = commands.arm1.twist_d;
            row.target_acc = commands.arm1.acc_d;
            row.arm2_position = {commands.arm2.pose_d[0], commands.arm2.pose_d[1], commands.arm2.pose_d[2]};
            row.arm2_hold_position_error = 0.0;
            row.arm2_hold_joint_error = 0.0;
            row.requested_Kx = machine.state() == Arm1State::TRANSPARENT
                                   ? K0
                                   : soft_stop::StopStiffness(ramp, K0, K_stop, row.stop_elapsed_s, row.stop_T_s);
            row.set_Kx = K_set;
            row.motion_ctrl = motion ? 1.0 : 0.0;
            row.state_stale_ticks = m.stale_ticks;
            row.stream_status = 1.0;
            row.fault_status = 0.0;
            soft_stop::WriteLogRow(*csv, row);
        }
    }
    const auto& plan = machine.plan();
    const double actual = soft_stop::Distance(hold_pose, latch_pose);
    const double final_error = soft_stop::Distance(arm.pose(), plan.stop_pose);
    std::cout << "  " << label << "：" << ticks << " 个周期，锁存 " << latches << " 次，|v0| = "
              << Num(soft_stop::Norm3(plan.v0)) << " m/s、|ω0| = " << Num(soft_stop::Norm3(plan.w0)) << " rad/s，T = "
              << Num(plan.T) << " s，计划停止距离 " << Num(plan.distance * 1000.0) << " mm、实际 "
              << Num(actual * 1000.0) << " mm；锁存后 " << Num(machine.hold_time() - plan.t0)
              << " s 进 HOLD；最大跟踪误差 " << Num(max_track * 1000.0) << " mm；结束时离终点 "
              << Num(final_error * 1000.0) << " mm；设刚度 " << ramp_calls << " 次\n";
    if (!error.empty()) {
        std::cout << "  模拟中状态机报错：" << error << "\n";
    }
    CHECK(error.empty());
    CHECK(latches == 1);
    CHECK(machine.state() == Arm1State::HOLD);
    CHECK(arm2_constant);
    CHECK(finite);
    CHECK(max_track < c.max_track_position_error);
    CHECK(final_error < c.hold_position_tol);
    CHECK(ramp ? ramp_calls > 2 : ramp_calls == 1);
}
}  // namespace

int main(int argc, char* argv[])
{
    std::string csv_path;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--csv" && i + 1 < argc) {
            csv_path = argv[++i];
        } else {
            std::cerr << "用法：arm1_soft_stop_test [--csv FILE.csv]\n";
            return 2;
        }
    }
    const std::pair<const char*, void (*)()> tests[] = {
        {"二次轨迹边界", TestTrajectoryBoundaries},
        {"t > T 恒定 HOLD", TestHoldAfterT},
        {"零速度", TestZeroVelocity},
        {"正负速度", TestDirections},
        {"锁住的轴", TestLockedAxes},
        {"停止时间 T 的规则（速度大也执行）", TestStopTimeRule},
        {"姿态", TestOrientation},
        {"Stop Sign 重复 100 次只锁存一次", TestLatchOnce},
        {"HOLD 不自动恢复透明", TestNoAutoResume},
        {"实测值无效或过期", TestBadMeasurement},
        {"时间异常", TestBadTime},
        {"停稳超时", TestSettleTimeout},
        {"跟踪误差", TestTrackError},
        {"刚度渐增公式", TestStiffness},
        {"速度滤波", TestTwistFilter},
        {"状态过期计数", TestStaleCounter},
        {"日志格式", TestLogFormat},
        {"无锁队列", TestRing},
        {"参数检查", TestConfigCheck},
    };
    for (const auto& [name, test] : tests) {
        std::cout << "== " << name << "\n";
        test();
    }
    std::cout << "== 1 kHz 闭环模拟（抖动、丢周期、切运动控制延迟 30 ms，ARM_2 指令逐字节不变）\n";
    std::ofstream csv;
    if (!csv_path.empty()) {
        csv.open(csv_path);
        if (!csv) {
            std::cerr << "打不开 " << csv_path << "\n";
            return 2;
        }
        csv.precision(9);
    }
    const Vec6 normal = {0.08, -0.05, 0.02, 0.1, -0.2, 0.15};  // 约 0.1 m/s、0.27 rad/s。
    const Vec6 fast = {0.5, -0.3, 0.2, 0.6, -0.5, 0.4};  // 约 0.62 m/s、0.88 rad/s：大速度也照样软停止。
    TestClosedLoopSim("一般速度、不开刚度渐增", false, normal, csv.is_open() ? &csv : nullptr);
    TestClosedLoopSim("一般速度、开刚度渐增", true, normal, nullptr);
    TestClosedLoopSim("大速度、不开刚度渐增", false, fast, nullptr);
    TestClosedLoopSim("大速度、开刚度渐增", true, fast, nullptr);
    if (csv.is_open()) {
        std::cout << "模拟日志已写入 " << csv_path << "\n";
    }
    std::cout << "\n" << g_checks << " 项检查，" << g_failures << " 项失败\n";
    return g_failures == 0 ? 0 : 1;
}
