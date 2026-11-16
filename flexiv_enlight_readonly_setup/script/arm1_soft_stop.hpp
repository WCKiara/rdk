// ARM_1 软停止：TRANSPARENT → SOFT_STOP → HOLD 状态机、二次位置减速轨迹、刚度渐增公式，以及 --log 的日志行和实时线程到
// 主线程的无锁队列。arm1_transparent_arm2_hold_rt_cartesian.cpp 和离线单元测试 arm1_soft_stop_test.cpp 共用。
// 只用标准库，不依赖 RDK、Eigen：单元测试在没装 RDK 的电脑上也能编。位姿都是 RDK 的 [x y z qw qx qy qz]，
// 速度 [vx vy vz wx wy wz]，世界坐标系，和 RobotStates::tcp_pose、tcp_twist、RtCartesianCmd 一致。
//
// 收到普通 Stop Sign 的那个实时周期（时刻 t0）锁存一次 ARM_1 实测位姿 p0、R0 和速度 v0、ω0。之后 t = now - t0，
// 用单调时钟的实际时间，不用周期数 × 1 ms：实时抖动、丢周期不改变制动时间。
//   位置 p_d = p0 + v0 (t - t²/2T)，速度 v_d = v0 (1 - t/T)，加速度 a_d = -v0/T；
//   t ≥ T 时停在 p_stop = p0 + v0 T/2，速度、加速度为 0。位置是二次函数，速度线性降到 0。
//   姿态是单位四元数，不能逐元素套用：旋转向量 θ = ω0 (t - t²/2T)，R_d = Exp(θ) R0（ω0 是世界系角速度，所以左乘），
//   ω_d = ω0 (1 - t/T)，α_d = -ω0/T，t ≥ T 时为 0。
//   T = max(T_min, |v0|/a_max, |ω0|/α_max)：满足减速度上限的最短时间。速度再大也照样执行，不设停止距离、转角、T 的上限：
//   速度大时 T 和停止距离 ½|v0|T = |v0|²/(2 a_max) 跟着变大。
// 锁存之后再收到 Stop Sign 不重新开始（不重置 t0、不重测 v0）；SOFT_STOP、HOLD 都不会自己回到 TRANSPARENT。
// 起点和终点的加速度有跳变，不是 jerk-limited 的最平顺停止。

#ifndef ARM1_SOFT_STOP_HPP_
#define ARM1_SOFT_STOP_HPP_

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <initializer_list>
#include <limits>
#include <ostream>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace soft_stop {
using Pose = std::array<double, 7>;  // [x y z qw qx qy qz]
using Vec3 = std::array<double, 3>;
using Vec6 = std::array<double, 6>;
using Quat = std::array<double, 4>;  // [w x y z]
using Joints = std::array<double, 7>;

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

enum class Arm1State
{
    TRANSPARENT = 0,  // 原来的透明手拖。
    SOFT_STOP = 1,    // 按二次位置曲线减速；t ≥ T 之后停在终点，等停稳。
    HOLD = 2,         // 已停稳，停在终点。
};

inline const char* StateName(Arm1State state)
{
    switch (state) {
        case Arm1State::TRANSPARENT:
            return "TRANSPARENT";
        case Arm1State::SOFT_STOP:
            return "SOFT_STOP";
        case Arm1State::HOLD:
            return "HOLD";
    }
    return "?";
}

struct CartesianCmd  // 和 rdk::RtCartesianCmd 一样的四项、一样的顺序：目标位姿、目标力、目标速度、目标加速度。
{
    Pose pose_d = {};
    Vec6 wrench_d = {};
    Vec6 twist_d = {};
    Vec6 acc_d = {};
};

struct StopConfig  // 软停止参数，单位都是 SI。
{
    std::array<bool, 3> free_axes = {true, true, true};  // TRANSPARENT 时能拖（力控）的 X Y Z；false = 锁住的轴，软停止时保持原来的目标。
    double max_linear_decel = 0.0;   // a_max [m/s²]
    double max_angular_decel = 0.0;  // α_max [rad/s²]
    double min_time = 0.0;           // T 的下限 [s]；没有上限。
    int max_stale_ticks = 0;         // 实测状态连续这么多个周期没更新算过期。
    double hold_linear_vel = 0.0;    // 停稳：实测线速度上限 [m/s]
    double hold_angular_vel = 0.0;   // 停稳：实测角速度上限 [rad/s]
    double hold_position_tol = 0.0;  // 停稳：离终点位置 [m]
    double hold_angle_tol = 0.0;     // 停稳：离终点姿态 [rad]
    double hold_max_force = 0.0;     // 停稳：末端外力 |f| 上限 [N]
    double hold_settle_time = 0.0;   // 停稳条件连续满足这么久才进 HOLD [s]
    double hold_timeout = 0.0;       // t ≥ T 之后最多等这么久 [s]
    double max_track_position_error = 0.0;  // SOFT_STOP、HOLD 中实测离目标位置的上限 [m]
    double max_track_angle_error = 0.0;     // 同上，姿态 [rad]
};

struct Measurement  // 一个实时周期 ARM_1 的实测值。
{
    Pose tcp_pose = {};
    Vec6 tcp_twist = {};  // 滤波后的 tcp_twist，见 TwistFilter。
    Vec6 tcp_wrench = {};
    int stale_ticks = 0;  // RobotStates::timestamp 连续多少个周期没变，见 StaleCounter。
};

struct StopPlan  // 锁存时定下的减速轨迹，之后不再改。
{
    double t0 = 0.0;        // 锁存时刻 [s]，单调时钟。
    double T = 0.0;         // 停止时间 [s]
    Vec3 p0 = {};           // 起点位置 [m]；锁住的轴是原来的目标。
    Vec3 v0 = {};           // 起点线速度 [m/s]；锁住的轴为 0。
    Quat q0 = {1.0, 0.0, 0.0, 0.0};  // 起点姿态。
    Vec3 w0 = {};           // 起点角速度 [rad/s]，世界系。
    double distance = 0.0;  // 计划停止距离 ½|v0|T [m]
    double angle = 0.0;     // 计划停止转角 ½|ω0|T [rad]
    Pose stop_pose = {};    // 终点位姿。
};

inline std::string Num(double value)  // 数字转成最短的文字，例如 10、0.05。
{
    std::ostringstream oss;
    oss << value;
    return oss.str();
}

template <size_t N>
bool AllFinite(const std::array<double, N>& values)
{
    return std::all_of(values.begin(), values.end(), [](double v) { return std::isfinite(v); });
}

template <size_t N>
constexpr std::array<double, N> Nans()  // 全是 NaN 的数组：日志里没有的数据。
{
    std::array<double, N> values = {};
    values.fill(kNaN);
    return values;
}

inline double Norm3(double x, double y, double z)
{
    return std::sqrt(x * x + y * y + z * z);
}

inline double Norm3(const Vec3& v)
{
    return Norm3(v[0], v[1], v[2]);
}

inline double LinearNorm(const Vec6& v)  // 前 3 个：线速度或力。
{
    return Norm3(v[0], v[1], v[2]);
}

inline double AngularNorm(const Vec6& v)  // 后 3 个：角速度或力矩。
{
    return Norm3(v[3], v[4], v[5]);
}

inline double Distance(const Pose& a, const Pose& b)  // 两个位姿的位置距离 [m]。
{
    return Norm3(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
}

// ================================ 四元数 [w x y z] ================================
inline Quat QuatOf(const Pose& pose)
{
    return {pose[3], pose[4], pose[5], pose[6]};
}

inline double QuatNorm(const Quat& q)
{
    return std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
}

inline Quat Normalized(const Quat& q)
{
    const double n = QuatNorm(q);
    return {q[0] / n, q[1] / n, q[2] / n, q[3] / n};
}

inline Quat Conjugate(const Quat& q)
{
    return {q[0], -q[1], -q[2], -q[3]};
}

inline Quat Multiply(const Quat& a, const Quat& b)  // a ⊗ b：先转 b、再转 a（都按世界系）。
{
    return {a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
        a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
        a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
        a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0]};
}

inline Quat Exp(const Vec3& r)  // 旋转向量 r（单位转轴 × 转角 [rad]）→ 单位四元数，即 SO(3) 指数映射。
{
    const double angle = Norm3(r);
    if (angle < 1e-9) {  // 小角度：sin(a/2)/a ≈ 1/2。
        return Normalized({1.0, 0.5 * r[0], 0.5 * r[1], 0.5 * r[2]});
    }
    const double s = std::sin(0.5 * angle) / angle;
    return {std::cos(0.5 * angle), s * r[0], s * r[1], s * r[2]};
}

inline Vec3 Log(const Quat& q)  // 单位四元数 → 旋转向量，转角在 [0, π]。q 和 -q 是同一个姿态，结果一样。
{
    const double sign = q[0] < 0.0 ? -1.0 : 1.0;
    const double s = Norm3(q[1], q[2], q[3]);
    const double angle = 2.0 * std::atan2(s, sign * q[0]);
    const double k = s < 1e-12 ? 2.0 : angle / s;  // 小角度：angle / sin(angle/2) ≈ 2。
    return {sign * k * q[1], sign * k * q[2], sign * k * q[3]};
}

inline double AngleBetween(const Quat& a, const Quat& b)  // 两个姿态之间的转角 [rad]，0 到 π。
{
    return Norm3(Log(Multiply(Normalized(b), Conjugate(Normalized(a)))));
}

// ================================ 轨迹 ================================
// 在锁存后 t 秒采样减速轨迹，写进 cmd，目标力为 0。t ≥ T 是终点，速度、加速度为 0；t < 0 按 0 算。
// 每 1 ms 在实时任务里调用，只用固定大小的数据，不分配内存。
inline void SampleStop(const StopPlan& plan, double t, CartesianCmd& cmd)
{
    const double tc = std::clamp(t, 0.0, plan.T);
    const bool moving = t < plan.T;
    const double travel = tc - tc * tc / (2.0 * plan.T);  // ∫₀ᵗ (1 - τ/T) dτ，从 0 增到 T/2。
    const double vel_scale = moving ? 1.0 - tc / plan.T : 0.0;
    const double acc_scale = moving ? -1.0 / plan.T : 0.0;
    for (size_t i = 0; i < 3; ++i) {
        cmd.pose_d[i] = plan.p0[i] + plan.v0[i] * travel;
        cmd.twist_d[i] = plan.v0[i] * vel_scale;
        cmd.acc_d[i] = plan.v0[i] * acc_scale;
        cmd.twist_d[3 + i] = plan.w0[i] * vel_scale;
        cmd.acc_d[3 + i] = plan.w0[i] * acc_scale;
    }
    const Quat q = Multiply(Exp({plan.w0[0] * travel, plan.w0[1] * travel, plan.w0[2] * travel}), plan.q0);
    std::copy(q.begin(), q.end(), cmd.pose_d.begin() + 3);
    cmd.wrench_d.fill(0.0);
}

// 用锁存时的实测值 m 规划减速轨迹，写进 plan。transparent_pose 是 TRANSPARENT 时的目标位姿：锁住的 X Y Z 保持它、不减速。
// 返回空串 = 可行；否则是不能软停止的原因：实测值无效或过期。速度再大也照样规划，不设停止距离上限。
inline std::string PlanStop(
    const StopConfig& c, const Pose& transparent_pose, const Measurement& m, double t0, StopPlan& plan)
{
    if (!AllFinite(m.tcp_pose) || !AllFinite(m.tcp_twist) || !AllFinite(m.tcp_wrench)) {
        return "ARM_1 实测位姿、速度或外力不是有限数";
    }
    if (const double n = QuatNorm(QuatOf(m.tcp_pose)); !(std::abs(n - 1.0) <= 0.01)) {
        return "ARM_1 实测四元数模长 " + Num(n) + "，不是单位四元数";
    }
    if (m.stale_ticks > c.max_stale_ticks) {
        return "ARM_1 实测状态已经 " + std::to_string(m.stale_ticks) + " 个周期没更新（RobotStates::timestamp 没变）";
    }
    plan.t0 = t0;
    for (size_t i = 0; i < 3; ++i) {
        plan.p0[i] = c.free_axes[i] ? m.tcp_pose[i] : transparent_pose[i];
        plan.v0[i] = c.free_axes[i] ? m.tcp_twist[i] : 0.0;
        plan.w0[i] = m.tcp_twist[3 + i];
    }
    plan.q0 = Normalized(QuatOf(m.tcp_pose));
    const double v = Norm3(plan.v0);
    const double w = Norm3(plan.w0);
    plan.T = std::max({c.min_time, v / c.max_linear_decel, w / c.max_angular_decel});
    plan.distance = 0.5 * v * plan.T;
    plan.angle = 0.5 * w * plan.T;
    CartesianCmd end;
    SampleStop(plan, plan.T, end);
    plan.stop_pose = end.pose_d;
    return "";
}

// 参数是否自洽；返回空串 = 正常。启动时检查一次。
inline std::string CheckConfig(const StopConfig& c)
{
    const std::initializer_list<std::pair<const char*, double>> positives = {
        {"max_linear_decel", c.max_linear_decel}, {"max_angular_decel", c.max_angular_decel},
        {"min_time", c.min_time}, {"hold_linear_vel", c.hold_linear_vel}, {"hold_angular_vel", c.hold_angular_vel},
        {"hold_position_tol", c.hold_position_tol}, {"hold_angle_tol", c.hold_angle_tol},
        {"hold_max_force", c.hold_max_force}, {"hold_settle_time", c.hold_settle_time},
        {"hold_timeout", c.hold_timeout}, {"max_track_position_error", c.max_track_position_error},
        {"max_track_angle_error", c.max_track_angle_error}};
    for (const auto& [name, value] : positives) {
        if (!(value > 0.0 && std::isfinite(value))) {
            return std::string(name) + " = " + Num(value) + "，要大于 0";
        }
    }
    if (c.max_stale_ticks < 1) {
        return "max_stale_ticks 要至少为 1";
    }
    if (!(c.hold_position_tol < c.max_track_position_error && c.hold_angle_tol < c.max_track_angle_error)) {
        return "停稳容差要小于跟踪误差上限，否则还没停稳就先停机了";
    }
    return "";
}

// ================================ 状态机 ================================
class Arm1StopMachine
{
public:
    // transparent_cmd：TRANSPARENT 时每周期发的 ARM_1 指令（原来的透明指令）。锁住的 X Y Z 软停止时保持它的位置。
    Arm1StopMachine(const StopConfig& config, const CartesianCmd& transparent_cmd)
    : config_(config)
    , transparent_cmd_(transparent_cmd)
    {
    }

    // 每个实时周期调用一次，算出 ARM_1 这一周期的指令 cmd。now：单调时钟 [s]。stop_requested：收到过普通 Stop Sign，
    // 之后一直为 true 也可以，只在 TRANSPARENT 时锁存一次。返回空串 = 正常；否则是不能开始或不能继续软停止的原因，
    // 调用方不要再发这个 cmd，走原有的保护停止。只用固定大小的数据；出错时才拼字符串。
    std::string Step(double now, bool stop_requested, const Measurement& m, CartesianCmd& cmd)
    {
        if (!std::isfinite(now)) {
            return "实时时钟读数不是有限数";
        }
        if (state_ == Arm1State::TRANSPARENT) {
            if (!stop_requested) {
                cmd = transparent_cmd_;
                return "";
            }
            if (auto error = PlanStop(config_, transparent_cmd_.pose_d, m, now, plan_); !error.empty()) {
                return "不能开始：" + error;
            }
            state_ = Arm1State::SOFT_STOP;
        }
        const double t = now - plan_.t0;
        SampleStop(plan_, t, cmd);
        if (!AllFinite(m.tcp_pose) || !AllFinite(m.tcp_twist) || !AllFinite(m.tcp_wrench)) {
            return std::string(StateName(state_)) + " 中 ARM_1 实测位姿、速度或外力不是有限数";
        }
        if (m.stale_ticks > config_.max_stale_ticks) {
            return std::string(StateName(state_)) + " 中 ARM_1 实测状态已经 " + std::to_string(m.stale_ticks)
                   + " 个周期没更新";
        }
        // 实测离目标太远（被推开、碰到东西、没切到运动控制）：不再硬拉，交给原有的保护停止。
        const double position_error = Distance(m.tcp_pose, cmd.pose_d);
        const double angle_error = AngleBetween(QuatOf(m.tcp_pose), QuatOf(cmd.pose_d));
        if (!(position_error <= config_.max_track_position_error) || !(angle_error <= config_.max_track_angle_error)) {
            return std::string(StateName(state_)) + " 中 ARM_1 离目标 " + Num(position_error * 1000.0) + " mm、"
                   + Num(angle_error) + " rad，超过跟踪误差上限 " + Num(config_.max_track_position_error * 1000.0)
                   + " mm、" + Num(config_.max_track_angle_error) + " rad";
        }
        if (state_ == Arm1State::SOFT_STOP && t >= plan_.T) {
            const bool still = LinearNorm(m.tcp_twist) <= config_.hold_linear_vel
                               && AngularNorm(m.tcp_twist) <= config_.hold_angular_vel
                               && position_error <= config_.hold_position_tol && angle_error <= config_.hold_angle_tol
                               && LinearNorm(m.tcp_wrench) <= config_.hold_max_force;
            if (!still) {
                still_since_ = kNaN;
            } else if (std::isnan(still_since_)) {
                still_since_ = now;
            }
            if (still && now - still_since_ >= config_.hold_settle_time) {
                state_ = Arm1State::HOLD;
                hold_time_ = now;
            } else if (t - plan_.T > config_.hold_timeout) {
                return "到 T 之后 " + Num(config_.hold_timeout) + " 秒仍没停稳："
                       + NotStillText(m, position_error, angle_error);
            }
        }
        return "";
    }

    Arm1State state() const { return state_; }
    const StopPlan& plan() const { return plan_; }  // 离开 TRANSPARENT 之后才有意义，之后不再改。
    double hold_time() const { return hold_time_; }  // 进 HOLD 的时刻 [s]；还没进是 NaN。

private:
    std::string NotStillText(const Measurement& m, double position_error, double angle_error) const  // 停稳条件的实测值。
    {
        return "线速度 " + Num(LinearNorm(m.tcp_twist)) + " m/s（要 ≤ " + Num(config_.hold_linear_vel) + "）、角速度 "
               + Num(AngularNorm(m.tcp_twist)) + " rad/s（要 ≤ " + Num(config_.hold_angular_vel) + "）、离终点 "
               + Num(position_error * 1000.0) + " mm（要 ≤ " + Num(config_.hold_position_tol * 1000.0) + "）、姿态差 "
               + Num(angle_error) + " rad（要 ≤ " + Num(config_.hold_angle_tol) + "）、外力 "
               + Num(LinearNorm(m.tcp_wrench)) + " N（要 ≤ " + Num(config_.hold_max_force) + "）";
    }

    StopConfig config_;
    CartesianCmd transparent_cmd_;
    Arm1State state_ = Arm1State::TRANSPARENT;
    StopPlan plan_;
    double still_since_ = kNaN;  // t ≥ T 之后连续满足停稳条件的起始时刻。
    double hold_time_ = kNaN;
};

// ================================ 刚度渐增（第二阶段） ================================
// K(t) = K0 + (Kf - K0)(t/T)²，t 夹在 [0, T]：t ≤ 0 是 K0，t ≥ T 正好是 Kf。结果夹在 K0、Kf 之间：
// 舍入误差不会让它比 Kf 大一点点，Kf = K_x_nom 时 SetCartesianImpedance 也不会因为超范围报错。
inline double StiffnessAt(double k0, double kf, double t, double T)
{
    const double s = T > 0.0 ? std::clamp(t / T, 0.0, 1.0) : 1.0;
    if (s >= 1.0) {
        return kf;
    }
    return std::clamp(k0 + (kf - k0) * s * s, std::min(k0, kf), std::max(k0, kf));
}

inline Vec6 StiffnessAt(const Vec6& k0, const Vec6& kf, double t, double T)
{
    Vec6 k;
    for (size_t i = 0; i < k.size(); ++i) {
        k[i] = StiffnessAt(k0[i], kf[i], t, T);
    }
    return k;
}

// 锁存后 ARM_1 应有的笛卡尔刚度：ramp 关时一步到 kf；开时按 StiffnessAt 渐增。
inline Vec6 StopStiffness(bool ramp, const Vec6& k0, const Vec6& kf, double t, double T)
{
    return ramp ? StiffnessAt(k0, kf, t, T) : kf;
}

// ================================ 实测值预处理 ================================
class TwistFilter  // 最近 samples 个周期 tcp_twist 的滑动平均：1 kHz 下 5 个样本，延迟约 2 ms。
{
public:
    static constexpr size_t kMaxSamples = 32;

    explicit TwistFilter(size_t samples)
    : samples_(std::clamp<size_t>(samples, 1, kMaxSamples))
    {
    }

    const Vec6& Update(const Vec6& twist)  // 放进一个新样本，返回平均值。
    {
        ring_[next_] = twist;
        next_ = (next_ + 1) % samples_;
        count_ = std::min(count_ + 1, samples_);
        average_.fill(0.0);
        for (size_t k = 0; k < count_; ++k) {
            for (size_t i = 0; i < average_.size(); ++i) {
                average_[i] += ring_[k][i];
            }
        }
        for (auto& v : average_) {
            v /= static_cast<double>(count_);
        }
        return average_;
    }

private:
    size_t samples_;
    std::array<Vec6, kMaxSamples> ring_ = {};
    size_t next_ = 0;
    size_t count_ = 0;
    Vec6 average_ = {};
};

class StaleCounter  // RobotStates::timestamp 连续多少个周期没变：判断实测状态是不是过期了。
{
public:
    int Update(int sec, int nsec)
    {
        if (sec == sec_ && nsec == nsec_) {
            ticks_ = ticks_ < std::numeric_limits<int>::max() ? ticks_ + 1 : ticks_;
        } else {
            sec_ = sec;
            nsec_ = nsec;
            ticks_ = 0;
        }
        return ticks_;
    }

private:
    int sec_ = -1;
    int nsec_ = -1;
    int ticks_ = 0;
};

// ================================ 日志 ================================
// --log 的一行：实时任务每周期填一行放进 SpscRing，主线程取出来写文件。没有的数据是 NaN。
struct LogRow
{
    double time_s = kNaN;          // ③ 开始后的秒数，单调时钟。
    double rt_cycle_us = kNaN;     // 和上一个实时周期的间隔 [µs]
    double stop_sign = kNaN;       // 到这个周期为止收到的 Stop Sign 次数。
    double arm1_state = kNaN;      // 0 TRANSPARENT，1 SOFT_STOP，2 HOLD。
    double stop_elapsed_s = kNaN;  // 锁存后的秒数 t。
    double stop_T_s = kNaN;        // 停止时间 T。
    Pose arm1_pose = Nans<7>();    // ARM_1 实测 tcp_pose。
    Vec6 arm1_twist = Nans<6>();   // ARM_1 实测 tcp_twist，没滤波。
    Vec3 arm1_force = Nans<3>();   // ARM_1 tcp_wrench 的力 [N]
    Pose target_pose = Nans<7>();  // 这个周期发给 ARM_1 的目标位姿、速度、加速度。
    Vec6 target_twist = Nans<6>();
    Vec6 target_acc = Nans<6>();
    Joints arm1_q = Nans<7>();
    Joints arm1_dtheta = Nans<7>();  // 电机侧关节速度，噪声小。
    Vec3 arm2_position = Nans<3>();
    Joints arm2_q = Nans<7>();
    Joints arm2_dtheta = Nans<7>();
    double arm2_hold_position_error = kNaN;  // ARM_2 末端离 hold 位置 [m]
    double arm2_hold_joint_error = kNaN;     // ARM_2 各关节离 hold 位置的最大值 [rad]
    Vec6 requested_Kx = Nans<6>();  // 这时 ARM_1 应有的笛卡尔刚度，见 StopStiffness。
    Vec6 set_Kx = Nans<6>();        // 主线程最近一次 SetCartesianImpedance(ARM_1) 完成的刚度。
    double impedance_latency_ms = kNaN;  // 最近一次 SetCartesianImpedance(ARM_1) 用时。
    double motion_ctrl = kNaN;      // 1 = 主线程已把 ARM_1 切回运动控制。
    double state_stale_ticks = kNaN;
    double stream_status = kNaN;    // 1 = 这个周期 StreamCartesianMotionForce 发出去了，0 = 出错。
    double fault_status = kNaN;     // 1 = robot.fault()。
};

inline void WriteLogHeader(std::ostream& out)  // 列的顺序和 WriteLogRow 一致。
{
    const auto names = [&out](const char* prefix, std::initializer_list<const char*> suffixes) {
        for (const char* suffix : suffixes) {
            out << ',' << prefix << suffix;
        }
    };
    const auto joints = [&out](const char* prefix) {
        for (int i = 1; i <= 7; ++i) {
            out << ',' << prefix << i;
        }
    };
    out << "time_s,rt_cycle_us,stop_sign,arm1_state,stop_elapsed_s,stop_T_s";
    names("arm1_", {"px", "py", "pz", "qw", "qx", "qy", "qz", "vx", "vy", "vz", "wx", "wy", "wz", "fx", "fy", "fz"});
    names("arm1_target_", {"px", "py", "pz", "qw", "qx", "qy", "qz", "vx", "vy", "vz", "wx", "wy", "wz", "ax", "ay",
                              "az", "alphax", "alphay", "alphaz"});
    joints("arm1_q");
    joints("arm1_dtheta");
    names("arm2_", {"px", "py", "pz"});
    joints("arm2_q");
    joints("arm2_dtheta");
    out << ",arm2_hold_position_error,arm2_hold_joint_error";
    names("requested_Kx_arm1_", {"x", "y", "z", "rx", "ry", "rz"});
    names("set_Kx_arm1_", {"x", "y", "z", "rx", "ry", "rz"});
    out << ",impedance_request_latency_ms,motion_ctrl,state_stale_ticks,stream_status,fault_status\n";
}

inline void WriteLogRow(std::ostream& out, const LogRow& r)
{
    const auto write = [&out](const auto& values) {
        for (double v : values) {
            out << ',' << v;
        }
    };
    out << r.time_s << ',' << r.rt_cycle_us << ',' << r.stop_sign << ',' << r.arm1_state << ',' << r.stop_elapsed_s
        << ',' << r.stop_T_s;
    write(r.arm1_pose);
    write(r.arm1_twist);
    write(r.arm1_force);
    write(r.target_pose);
    write(r.target_twist);
    write(r.target_acc);
    write(r.arm1_q);
    write(r.arm1_dtheta);
    write(r.arm2_position);
    write(r.arm2_q);
    write(r.arm2_dtheta);
    out << ',' << r.arm2_hold_position_error << ',' << r.arm2_hold_joint_error;
    write(r.requested_Kx);
    write(r.set_Kx);
    out << ',' << r.impedance_latency_ms << ',' << r.motion_ctrl << ',' << r.state_stale_ticks << ','
        << r.stream_status << ',' << r.fault_status << '\n';
}

// 单生产者（实时线程）、单消费者（主线程）的无锁队列：容量在构造时分配，之后不再分配内存；满了 TryPush 返回 false，
// 生产者丢掉这一行、不等待。
template <typename T>
class SpscRing
{
public:
    explicit SpscRing(size_t capacity)
    : buffer_(capacity + 1)
    {
    }

    bool TryPush(const T& item)
    {
        const size_t head = head_.load(std::memory_order_relaxed);
        const size_t next = (head + 1) % buffer_.size();
        if (next == tail_.load(std::memory_order_acquire)) {
            return false;
        }
        buffer_[head] = item;
        head_.store(next, std::memory_order_release);
        return true;
    }

    bool TryPop(T& item)
    {
        const size_t tail = tail_.load(std::memory_order_relaxed);
        if (tail == head_.load(std::memory_order_acquire)) {
            return false;
        }
        item = buffer_[tail];
        tail_.store((tail + 1) % buffer_.size(), std::memory_order_release);
        return true;
    }

private:
    std::vector<T> buffer_;
    std::atomic<size_t> head_ = {0};  // 下一个写的位置，只有生产者改。
    std::atomic<size_t> tail_ = {0};  // 下一个读的位置，只有消费者改。
};
}  // namespace soft_stop

#endif  // ARM1_SOFT_STOP_HPP_
