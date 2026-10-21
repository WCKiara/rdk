#pragma once

// Local trajectory helpers: no robot connection or command APIs.
#include <Eigen/Geometry>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace single_arm_rt {
using Pose = std::array<double, 7>;
constexpr double kPi = 3.14159265358979323846;

template <class Values> void Finite(const Values& values)
{
    for (double value : values) {
        if (!std::isfinite(value)) throw std::invalid_argument("Non-finite state or argument");
    }
}

inline Eigen::Quaterniond Rotation(const Pose& pose)
{
    Finite(pose);
    Eigen::Quaterniond rotation(pose[3], pose[4], pose[5], pose[6]);
    if (!std::isfinite(rotation.norm()) || rotation.norm() < 1e-8)
        throw std::invalid_argument("Invalid TCP quaternion norm");
    return rotation.normalized();
}

inline double OrientationErrorDeg(const Pose& first, const Pose& second)
{
    // Quaternion signs are equivalent; never force a sign change at qw == 0.
    return Rotation(first).angularDistance(Rotation(second)) * 180.0 / kPi;
}

inline void CheckStartPose(const Pose& captured_start, const Pose& actual)
{
    const double angle = OrientationErrorDeg(captured_start, actual);
    Eigen::Vector3d delta;
    for (size_t i = 0; i < 3; ++i) delta[i] = actual[i] - captured_start[i];
    const double distance = delta.norm();
    if (distance > 0.001 || angle > 1.0) {
        std::ostringstream message;
        message << std::setprecision(12)
                << "ARM_1 moved from captured Home/start before first target: position_error="
                << distance << " m (limit 0.001 m), orientation_error=" << angle
                << " deg (limit 1 deg); no streaming command sent";
        throw std::runtime_error(message.str());
    }
}

struct Bounds {
    std::array<double, 3> lower {0.35, 0.15, 0.04727903753519058};
    std::array<double, 3> upper {0.46, 0.25, 0.18};
    double margin = 0.005;
    void Validate() const
    {
        Finite(lower);
        Finite(upper);
        if (!std::isfinite(margin) || margin < 0) throw std::invalid_argument("Invalid limit margin");
        for (size_t i = 0; i < 3; ++i) {
            if (lower[i] + margin >= upper[i] - margin)
                throw std::invalid_argument("Position bounds have no inner workspace");
        }
    }
    void Check(const Pose& pose) const
    {
        Rotation(pose);
        for (size_t i = 0; i < 3; ++i) {
            if (pose[i] <= lower[i] + margin || pose[i] >= upper[i] - margin)
                throw std::runtime_error("TCP reached position boundary / inner margin");
        }
    }
};

inline void GuardArm2(const std::vector<double>& baseline, const std::vector<double>& actual,
    const std::vector<double>& velocity, const char* phase = "monitor")
{
    if (baseline.size() != 7 || baseline.size() != actual.size() || actual.size() != velocity.size())
        throw std::runtime_error("Missing or inconsistent ARM_2 joint states");
    Finite(baseline);
    Finite(actual);
    Finite(velocity);
    for (size_t i = 0; i < baseline.size(); ++i) {
        const double drift = actual[i] - baseline[i];
        if (std::abs(drift) > 0.001 || std::abs(velocity[i]) > 0.01) {
            std::ostringstream message;
            message << std::setprecision(9) << "ARM_2 stationary check failed [" << phase
                    << "]: joint=" << i + 1 << ", delta_q=" << drift
                    << " rad (limit abs=0.001 rad), speed_100ms=" << velocity[i]
                    << " rad/s (limit abs=0.01 rad/s); speed_source=position_window_100ms; "
                       "ARM_2 is monitored only; "
                       "no motion target is sent to it";
            throw std::runtime_error(message.str());
        }
    }
}

// The SDK documents raw link-encoder dq as noisy. Estimate motion from q over
// a 100 ms trailing window instead; the baseline position guard stays immediate.
class Arm2StateGuard {
    static constexpr int64_t kWindowNs = 100000000;
    static constexpr size_t kCapacity = 128;
    struct Sample { int64_t stamp; std::array<double, 7> q; };
    std::array<Sample, kCapacity> history_ {};
    std::vector<double> baseline_, speed_ = std::vector<double>(7, 0.0);
    size_t head_ = 0, count_ = 1;
    bool ready_ = false;

    void Store(size_t index, int64_t stamp, const std::vector<double>& q)
    {
        history_[index].stamp = stamp;
        std::copy(q.begin(), q.end(), history_[index].q.begin());
    }
public:
    Arm2StateGuard(const std::vector<double>& baseline, int64_t timestamp_ns)
    : baseline_(baseline)
    {
        if (timestamp_ns < 0) throw std::runtime_error("Invalid ARM_2 timestamp");
        GuardArm2(baseline_, baseline_, speed_, "startup snapshot");
        Store(0, timestamp_ns, baseline_);
    }

    bool Ready() const { return ready_; }

    void Check(const std::vector<double>& q, const std::vector<double>& raw_dq,
        int64_t timestamp_ns, const char* phase = "monitor")
    {
        if (q.size() != 7 || raw_dq.size() != 7)
            throw std::runtime_error("Missing or inconsistent ARM_2 joint states");
        for (size_t i = 0; i < raw_dq.size(); ++i) {
            if (!std::isfinite(raw_dq[i])) {
                std::ostringstream message;
                message << "Invalid ARM_2 raw dq [" << phase << "]: joint=" << i + 1
                        << ", raw_dq=" << raw_dq[i];
                throw std::runtime_error(message.str());
            }
        }
        GuardArm2(baseline_, q, speed_, phase); // Position is checked even during warmup.
        const auto newest = (head_ + count_ - 1) % kCapacity;
        if (timestamp_ns < history_[newest].stamp)
            throw std::runtime_error("ARM_2 state timestamp regressed");
        if (timestamp_ns == history_[newest].stamp) return;
        const auto cutoff = timestamp_ns - kWindowNs;
        // Keep one sample at/before the cutoff, then all more recent samples.
        while (count_ > 1 && history_[(head_ + 1) % kCapacity].stamp <= cutoff) {
            head_ = (head_ + 1) % kCapacity;
            --count_;
        }
        if (count_ == kCapacity)
            throw std::runtime_error("ARM_2 position window exceeded sample capacity");
        Store((head_ + count_) % kCapacity, timestamp_ns, q);
        ++count_;
        const auto& anchor = history_[head_];
        const int64_t elapsed_ns = timestamp_ns - anchor.stamp;
        if (elapsed_ns >= kWindowNs) {
            const double elapsed = static_cast<double>(elapsed_ns) * 1e-9;
            for (size_t i = 0; i < speed_.size(); ++i) speed_[i] = (q[i] - anchor.q[i]) / elapsed;
            ready_ = true;
        }
        GuardArm2(baseline_, q, speed_, phase);
    }
};

struct Plan {
    Pose start, target;
    double duration;
    Plan(const Pose& initial, const std::array<double, 3>& delta,
        const std::array<double, 3>& tilt_deg, double requested_duration, double linear_vel,
        double angular_vel, double linear_acc, double angular_acc, const Bounds& bounds)
    : start(initial), target(initial), duration(requested_duration)
    {
        bounds.Validate();
        bounds.Check(start);
        Finite(delta);
        Finite(tilt_deg);
        const std::array<double, 5> positive {
            requested_duration, linear_vel, angular_vel, linear_acc, angular_acc};
        Finite(positive);
        for (double value : positive) {
            if (value <= 0) throw std::invalid_argument("Duration, velocities and accelerations must be positive");
        }
        for (size_t i = 0; i < 3; ++i) target[i] += delta[i];
        const auto q0 = Rotation(start);
        // Same convention as scipy Rotation.from_euler("xyz"): Rz * Ry * Rx.
        const Eigen::Quaterniond increment
            = Eigen::AngleAxisd(tilt_deg[2] * kPi / 180, Eigen::Vector3d::UnitZ())
              * Eigen::AngleAxisd(tilt_deg[1] * kPi / 180, Eigen::Vector3d::UnitY())
              * Eigen::AngleAxisd(tilt_deg[0] * kPi / 180, Eigen::Vector3d::UnitX());
        const Eigen::Quaterniond q1 = (q0 * increment).normalized();
        start[3] = q0.w(); start[4] = q0.x(); start[5] = q0.y(); start[6] = q0.z();
        target[3] = q1.w(); target[4] = q1.x(); target[5] = q1.y(); target[6] = q1.z();
        bounds.Check(target);
        const double distance = Eigen::Vector3d(delta[0], delta[1], delta[2]).norm();
        const double angle = q0.angularDistance(q1);
        const double peak_acc = 10 / std::sqrt(3.0);
        duration = std::max({duration, 1.875 * distance / linear_vel,
            1.875 * angle / angular_vel, std::sqrt(peak_acc * distance / linear_acc),
            std::sqrt(peak_acc * angle / angular_acc)});
        if (!std::isfinite(duration)) throw std::invalid_argument("Trajectory duration overflow");
    }
    Pose At(double elapsed) const
    {
        if (!std::isfinite(elapsed)) throw std::invalid_argument("Invalid trajectory time");
        const double u = std::clamp(elapsed / duration, 0.0, 1.0);
        const double s = u * u * u * (10 + u * (-15 + 6 * u));
        Pose result = start;
        for (size_t i = 0; i < 3; ++i) result[i] += s * (target[i] - start[i]);
        const Eigen::Quaterniond q = Rotation(start).slerp(s, Rotation(target)).normalized();
        result[3] = q.w(); result[4] = q.x(); result[5] = q.y(); result[6] = q.z();
        return result;
    }
    bool Reached(const Pose& actual, double position_tol = 0.001,
        double angle_tol_rad = kPi / 180) const
    {
        const auto q = Rotation(actual);
        Eigen::Vector3d error;
        for (size_t i = 0; i < 3; ++i) error[i] = actual[i] - target[i];
        return error.norm() <= position_tol && q.angularDistance(Rotation(target)) <= angle_tol_rad;
    }
};
} // namespace single_arm_rt
