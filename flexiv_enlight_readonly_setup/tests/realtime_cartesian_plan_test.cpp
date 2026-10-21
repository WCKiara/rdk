// Offline trajectory/guard tests. This translation unit never loads the robot SDK.
#include "../third_party/flexiv_rdk/example/single_arm_realtime_plan.hpp"

#include <Eigen/Geometry>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <functional>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
using single_arm_rt::Bounds;
using single_arm_rt::Plan;
using single_arm_rt::Pose;
constexpr double kPi = 3.14159265358979323846;
const Pose kStart{0.40, 0.20, 0.12, 1.0, 0.0, 0.0, 0.0};

void Require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}

template <typename Callback>
void Rejects(Callback callback, const std::string& message) {
    bool rejected = false;
    try {
        callback();
    } catch (const std::exception&) {
        rejected = true;
    }
    Require(rejected, message);
}

Eigen::Quaterniond Quaternion(const Pose& pose) {
    return Eigen::Quaterniond(pose[3], pose[4], pose[5], pose[6]);
}

Eigen::Vector3d Position(const Pose& pose) {
    return Eigen::Vector3d(pose[0], pose[1], pose[2]);
}

Eigen::Vector3d RotationVector(Eigen::Quaterniond rotation) {
    rotation.normalize();
    if (rotation.w() < 0.0) rotation.coeffs() *= -1.0;
    const double sine = rotation.vec().norm();
    if (sine < 1e-14) return 2.0 * rotation.vec();
    return (2.0 * std::atan2(sine, rotation.w()) / sine) * rotation.vec();
}

double Angle(const Pose& first, const Pose& second) {
    return RotationVector(Quaternion(first).conjugate() * Quaternion(second)).norm();
}

void SamePose(const Pose& actual, const Pose& expected, const std::string& message) {
    Require((Position(actual) - Position(expected)).norm() < 1e-11, message + " position");
    Require(Angle(actual, expected) < 1e-11, message + " rotation");
}

Plan MakePlan(const Pose& start = kStart,
              const std::array<double, 3>& delta = {0.01, 0.0, -0.01},
              const std::array<double, 3>& tilt = {0.0, 15.0, 0.0},
              double requested_duration = 5.0, double linear_vel = 0.01,
              double angular_vel = 0.15, double linear_acc = 0.1,
              double angular_acc = 0.3, const Bounds& bounds = Bounds{}) {
    return Plan(start, delta, tilt, requested_duration, linear_vel, angular_vel,
                linear_acc, angular_acc, bounds);
}

void TestEndpointsAndToolRelativeTilt() {
    Pose start = kStart;
    const Eigen::Quaterniond initial(Eigen::AngleAxisd(kPi / 2, Eigen::Vector3d::UnitZ()));
    start[3] = initial.w();
    start[4] = initial.x();
    start[5] = initial.y();
    start[6] = initial.z();
    const std::array<double, 3> tilt{10.0, 15.0, -20.0};
    const Plan plan = MakePlan(start, {0.01, 0.0, -0.01}, tilt);
    SamePose(plan.At(0.0), start, "start endpoint");
    SamePose(plan.At(plan.duration), plan.target, "target endpoint");
    Require((Position(plan.target) - Eigen::Vector3d(0.41, 0.20, 0.11)).norm() < 1e-12,
            "delta must remain in world coordinates");
    const Eigen::Quaterniond expected = initial
        * Eigen::AngleAxisd(tilt[2] * kPi / 180.0, Eigen::Vector3d::UnitZ())
        * Eigen::AngleAxisd(tilt[1] * kPi / 180.0, Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(tilt[0] * kPi / 180.0, Eigen::Vector3d::UnitX());
    Require(RotationVector(expected.conjugate() * Quaternion(plan.target)).norm() < 1e-11,
            "tilt must be relative to the initial tool frame with xyz Euler convention");
    const Pose midpoint = plan.At(plan.duration / 2.0);
    Require((Position(midpoint) - Eigen::Vector3d(0.405, 0.20, 0.115)).norm() < 1e-11,
            "trajectory midpoint");
    Require(std::abs(Angle(start, midpoint) - 0.5 * Angle(start, plan.target)) < 1e-11,
            "quaternion interpolation midpoint");
    Require(plan.duration >= 5.0, "requested duration is a lower bound");
}

void TestHomeQuaternionStartsAtMeasuredOrientation() {
    Pose home{0.4011, 0.1996, 0.1213, 0.001538137, -0.000788480, 0.999998495, 0.000152865};
    const auto normalized = Quaternion(home).normalized();
    home[3] = normalized.w(); home[4] = normalized.x();
    home[5] = normalized.y(); home[6] = normalized.z();
    for (const double sign : {1.0, -1.0}) {
        Pose signed_home = home;
        for (size_t i = 3; i < signed_home.size(); ++i) signed_home[i] *= sign;
        const Plan plan = MakePlan(signed_home, {0.01, 0.0, -0.01}, {0.0, 15.0, 0.0});
        const Pose first = plan.At(0.0);
        for (size_t i = 0; i < first.size(); ++i)
            Require(std::abs(first[i] - signed_home[i]) < 1e-14,
                    "first Home packet must preserve measured position/quaternion sign");
        Require(std::abs(Quaternion(first).w()) < 0.01
                && std::abs(Quaternion(first).y()) > 0.99,
                "near-180-degree Home orientation was replaced with identity");
        for (int milliseconds = 1; milliseconds <= 4; ++milliseconds) {
            Require(Angle(signed_home, plan.At(milliseconds * 0.001)) < 1e-7,
                    "Home trajectory rotated abruptly in the first four milliseconds");
        }
        const Pose final = plan.At(plan.duration);
        Require(std::abs(Angle(signed_home, final) - 15.0 * kPi / 180.0) < 1e-12,
                "15-degree relative tilt was not preserved from Home");
        const auto expected = Quaternion(signed_home)
            * Eigen::Quaterniond(Eigen::AngleAxisd(15.0 * kPi / 180.0, Eigen::Vector3d::UnitY()));
        Require(Quaternion(final).coeffs().dot(expected.coeffs()) > 1.0 - 1e-12,
                "Home target quaternion changed sign or lost tool-relative composition");
        Require(std::abs(final[0] - home[0] - 0.01) < 1e-12
                && std::abs(final[1] - home[1]) < 1e-12
                && std::abs(final[2] - home[2] + 0.01) < 1e-12,
                "Home delta was not applied in world coordinates");
    }
}

void TestCapturedStartPoseGuard() {
    Pose home{0.4011, 0.1996, 0.1213, 0.001538137, -0.000788480, 0.999998495, 0.000152865};
    const auto normalized = Quaternion(home).normalized();
    home[3] = normalized.w(); home[4] = normalized.x();
    home[5] = normalized.y(); home[6] = normalized.z();
    Pose actual = home;
    for (size_t i = 3; i < actual.size(); ++i) actual[i] *= -1;
    single_arm_rt::CheckStartPose(home, actual);
    Require(single_arm_rt::OrientationErrorDeg(home, actual) < 1e-10,
            "quaternion sign changed measured orientation error");
    actual = home;
    actual[0] += 0.0009;
    single_arm_rt::CheckStartPose(home, actual);
    actual[0] = home[0] + 0.0011;
    Rejects([&] { single_arm_rt::CheckStartPose(home, actual); },
            "captured start guard accepted more than 1 mm displacement");
    for (const double angle : {0.9, 1.1}) {
        actual = home;
        const auto q = Quaternion(home)
            * Eigen::Quaterniond(Eigen::AngleAxisd(angle * kPi / 180, Eigen::Vector3d::UnitY()));
        actual[3] = q.w(); actual[4] = q.x(); actual[5] = q.y(); actual[6] = q.z();
        if (angle < 1) single_arm_rt::CheckStartPose(home, actual);
        else Rejects([&] { single_arm_rt::CheckStartPose(home, actual); },
                "captured start guard accepted more than 1 degree orientation change");
    }
}

void CheckNumericalLimits(double linear_vel, double angular_vel,
                          double linear_acc, double angular_acc) {
    const Plan plan = MakePlan(kStart, {0.02, 0.01, -0.02}, {20.0, -40.0, 60.0},
                               0.1, linear_vel, angular_vel, linear_acc, angular_acc);
    Require(std::isfinite(plan.duration) && plan.duration > 0.1,
            "motion must stretch its duration to satisfy velocity/acceleration limits");
    constexpr int samples = 5000;
    const double dt = plan.duration / samples;
    Pose previous = plan.At(0.0);
    Eigen::Vector3d previous_linear_velocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d previous_angular_velocity = Eigen::Vector3d::Zero();
    double max_linear_velocity = 0.0, max_angular_velocity = 0.0;
    double max_linear_acceleration = 0.0, max_angular_acceleration = 0.0;
    for (int sample = 1; sample <= samples; ++sample) {
        const Pose current = plan.At(sample * dt);
        Bounds{}.Check(current);
        Require(std::abs(Quaternion(current).norm() - 1.0) < 1e-10,
                "all interpolated quaternions must remain normalized");
        const Eigen::Vector3d linear_velocity = (Position(current) - Position(previous)) / dt;
        const Eigen::Vector3d angular_velocity = RotationVector(
            Quaternion(previous).conjugate() * Quaternion(current)) / dt;
        max_linear_velocity = std::max(max_linear_velocity, linear_velocity.norm());
        max_angular_velocity = std::max(max_angular_velocity, angular_velocity.norm());
        if (sample > 1) {
            max_linear_acceleration = std::max(max_linear_acceleration,
                (linear_velocity - previous_linear_velocity).norm() / dt);
            max_angular_acceleration = std::max(max_angular_acceleration,
                (angular_velocity - previous_angular_velocity).norm() / dt);
        }
        previous = current;
        previous_linear_velocity = linear_velocity;
        previous_angular_velocity = angular_velocity;
    }
    constexpr double numerical_slack = 1.0001;
    Require(max_linear_velocity <= linear_vel * numerical_slack,
            "numerical Cartesian speed exceeded linear limit");
    Require(max_angular_velocity <= angular_vel * numerical_slack,
            "numerical orientation speed exceeded angular limit");
    Require(max_linear_acceleration <= linear_acc * numerical_slack,
            "numerical Cartesian acceleration exceeded linear limit");
    Require(max_angular_acceleration <= angular_acc * numerical_slack,
            "numerical orientation acceleration exceeded angular limit");
    Require((Position(plan.At(dt)) - Position(plan.At(0))).norm() / dt < linear_vel * 1e-5,
            "linear speed should begin at zero");
    Require(Angle(plan.At(0), plan.At(dt)) / dt < angular_vel * 1e-5,
            "angular speed should begin at zero");
    Require((Position(plan.At(plan.duration)) - Position(plan.At(plan.duration - dt))).norm()
                / dt < linear_vel * 1e-5,
            "linear speed should end at zero");
    Require(Angle(plan.At(plan.duration - dt), plan.At(plan.duration)) / dt
                < angular_vel * 1e-5,
            "angular speed should end at zero");
}

void TestNumericalVelocityAndAccelerationLimits() {
    // Each limit is independently dominant in one scenario. A missing acceleration
    // constraint cannot pass merely because a low velocity already slowed the plan.
    CheckNumericalLimits(0.003, 0.08, 0.004, 0.03);
    CheckNumericalLimits(0.0005, 10.0, 1.0, 10.0);
    CheckNumericalLimits(1.0, 0.02, 1.0, 10.0);
    CheckNumericalLimits(1.0, 10.0, 0.00005, 10.0);
    CheckNumericalLimits(1.0, 10.0, 1.0, 0.0005);
}

void TestBoundsAndMargin() {
    Bounds bounds;
    bounds.Validate();
    Pose edge = kStart;
    edge[0] = bounds.lower[0] + bounds.margin + 1e-7;
    bounds.Check(edge);
    edge[0] = bounds.lower[0] + bounds.margin;
    Rejects([&] { bounds.Check(edge); }, "lower safety margin must be enforced");
    edge = kStart;
    edge[2] = bounds.upper[2] - bounds.margin - 1e-7;
    bounds.Check(edge);
    edge[2] = bounds.upper[2] - bounds.margin;
    Rejects([&] { bounds.Check(edge); }, "upper safety margin must be enforced");
    Rejects([] { MakePlan(kStart, {0.1, 0.0, 0.0}); }, "out-of-bounds target must fail");
    Pose outside = kStart;
    outside[1] = 0.30;
    Rejects([&] { MakePlan(outside); }, "out-of-bounds start must fail");
    Bounds invalid = bounds;
    invalid.margin = -0.001;
    Rejects([&] { invalid.Validate(); }, "negative margin must fail");
    invalid = bounds;
    invalid.lower[0] = invalid.upper[0];
    Rejects([&] { invalid.Validate(); }, "empty bounds must fail");
    invalid = bounds;
    invalid.margin = 1.0;
    Rejects([&] { invalid.Validate(); }, "margin that consumes the bounds must fail");
    invalid = bounds;
    invalid.upper[0] = std::numeric_limits<double>::infinity();
    Rejects([&] { invalid.Validate(); }, "infinite bounds must fail");
}

void TestInvalidAndStationaryInputs() {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    Pose invalid = kStart;
    invalid[3] = 0.0;
    Rejects([&] { MakePlan(invalid); }, "zero quaternion must fail");
    invalid = kStart;
    invalid[0] = nan;
    Rejects([&] { MakePlan(invalid); }, "nonfinite initial position must fail");
    invalid = kStart;
    invalid[5] = nan;
    Rejects([&] { MakePlan(invalid); }, "nonfinite quaternion must fail");
    invalid = kStart;
    invalid[3] = std::numeric_limits<double>::max();
    Rejects([&] { MakePlan(invalid); }, "finite quaternion with overflowing norm must fail");
    Rejects([&] { MakePlan(kStart, {nan, 0.0, 0.0}); }, "nonfinite delta must fail");
    Rejects([&] { MakePlan(kStart, {0.0, 0.0, 0.0}, {0.0, nan, 0.0}); },
            "nonfinite tilt must fail");
    const std::array<double, 5> valid_limits{5.0, 0.01, 0.15, 0.1, 0.3};
    for (std::size_t index = 0; index < valid_limits.size(); ++index) {
        for (const double invalid_limit : {0.0, -1.0, nan,
                                            std::numeric_limits<double>::infinity()}) {
            auto limits = valid_limits;
            limits[index] = invalid_limit;
            Rejects([&] { MakePlan(kStart, {0.01, 0.0, -0.01}, {0.0, 15.0, 0.0},
                                  limits[0], limits[1], limits[2], limits[3], limits[4]); },
                    "invalid duration/velocity/acceleration must fail");
        }
    }
    const Plan stationary = MakePlan(kStart, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0});
    SamePose(stationary.At(stationary.duration / 2.0), kStart, "stationary midpoint");
    SamePose(stationary.At(stationary.duration), kStart, "stationary endpoint");
    Require(std::isfinite(stationary.duration), "stationary duration must remain finite");
}

void TestReachedUsesPositionAndOrientation() {
    const Plan plan = MakePlan();
    Require(plan.Reached(plan.target), "target must count as reached");
    Require(!plan.Reached(plan.start), "initial pose must not count as target");
    Pose actual = plan.target;
    for (std::size_t i = 3; i < actual.size(); ++i) actual[i] *= -1.0;
    Require(plan.Reached(actual), "quaternion sign must not change target acceptance");
    actual = plan.target;
    actual[0] += 0.0011;
    Require(!plan.Reached(actual), "position beyond tolerance must not count as reached");
    const Eigen::Quaterniond rotated = Quaternion(plan.target)
        * Eigen::AngleAxisd(1.1 * kPi / 180.0, Eigen::Vector3d::UnitZ());
    actual = plan.target;
    actual[3] = rotated.w(); actual[4] = rotated.x();
    actual[5] = rotated.y(); actual[6] = rotated.z();
    Require(!plan.Reached(actual), "orientation beyond tolerance must not count as reached");
}

void TestArm2Guard() {
    const std::vector<double> baseline(7, 0.0);
    std::vector<double> actual = baseline, velocity(7, 0.0);
    single_arm_rt::GuardArm2(baseline, actual, velocity);
    actual[2] = 0.0009;
    velocity[5] = -0.009;
    single_arm_rt::GuardArm2(baseline, actual, velocity);
    actual[2] = 0.0011;
    Rejects([&] { single_arm_rt::GuardArm2(baseline, actual, velocity); },
            "ARM_2 displacement above 0.001 rad must stop");
    actual = baseline;
    velocity[5] = -0.0101;
    Rejects([&] { single_arm_rt::GuardArm2(baseline, actual, velocity); },
            "ARM_2 speed above 0.01 rad/s must stop");
    velocity.assign(7, 0.0);
    actual[0] = std::numeric_limits<double>::quiet_NaN();
    Rejects([&] { single_arm_rt::GuardArm2(baseline, actual, velocity); },
            "nonfinite ARM_2 positions must stop");
    actual = baseline;
    velocity[0] = std::numeric_limits<double>::quiet_NaN();
    Rejects([&] { single_arm_rt::GuardArm2(baseline, actual, velocity); },
            "nonfinite ARM_2 speed must stop");
    velocity.assign(7, 0.0);
    auto invalid_baseline = baseline;
    invalid_baseline[1] = std::numeric_limits<double>::quiet_NaN();
    Rejects([&] { single_arm_rt::GuardArm2(invalid_baseline, actual, velocity); },
            "nonfinite ARM_2 baseline must stop");
    actual.resize(6);
    Rejects([&] { single_arm_rt::GuardArm2(baseline, actual, velocity); },
            "ARM_2 position size mismatch must stop");
    actual = baseline;
    velocity.resize(6);
    Rejects([&] { single_arm_rt::GuardArm2(baseline, actual, velocity); },
            "ARM_2 velocity size mismatch must stop");
    Rejects([] { single_arm_rt::GuardArm2({}, {}, {}); }, "missing ARM_2 state must stop");
    const std::vector<double> too_few_joints(6, 0.0);
    Rejects([&] { single_arm_rt::GuardArm2(too_few_joints, too_few_joints, too_few_joints); },
            "matching ARM_2 vectors with fewer than seven joints must stop");
}

void TestArm2GuardDiagnostics() {
    const std::vector<double> baseline(7, 0.0);
    std::vector<double> actual = baseline, velocity(7, 0.0);
    auto diagnostic = [&](const char* phase) {
        try {
            single_arm_rt::GuardArm2(baseline, actual, velocity, phase);
        } catch (const std::runtime_error& error) {
            return std::string(error.what());
        }
        throw std::runtime_error("expected an ARM_2 diagnostic");
    };

    actual[2] = -0.002;
    const auto drift = diagnostic("real-time motion");
    for (const auto* detail : {"[real-time motion]", "joint=3", "delta_q=-0.002",
             "speed_100ms=0", "limit abs=0.001 rad", "limit abs=0.01 rad/s",
             "speed_source=position_window_100ms",
             "ARM_2 is monitored only; no motion target is sent to it"}) {
        Require(drift.find(detail) != std::string::npos,
                "drift diagnostic omitted: " + std::string(detail));
    }

    actual = baseline;
    velocity[5] = -0.02;
    const auto speed = diagnostic("startup snapshot");
    for (const auto* detail : {"[startup snapshot]", "joint=6", "delta_q=0", "speed_100ms=-0.02"}) {
        Require(speed.find(detail) != std::string::npos,
                "speed diagnostic omitted: " + std::string(detail));
    }
}

void TestArm2GuardExactLimits() {
    const std::vector<double> baseline(7, 0.0);
    std::vector<double> actual = baseline, velocity(7, 0.0);
    actual[2] = 0.001;
    velocity[5] = -0.01;
    single_arm_rt::GuardArm2(baseline, actual, velocity);
    actual[2] = std::nextafter(0.001, std::numeric_limits<double>::infinity());
    Rejects([&] { single_arm_rt::GuardArm2(baseline, actual, velocity); },
            "even the first representable drift above 0.001 rad must stop");
    actual = baseline;
    velocity[5] = std::nextafter(-0.01, -std::numeric_limits<double>::infinity());
    Rejects([&] { single_arm_rt::GuardArm2(baseline, actual, velocity); },
            "even the first representable speed beyond -0.01 rad/s must stop");
}

void TestArm2WindowWarmupAndDuplicateTimestamps() {
    const std::vector<double> baseline(7, 0.0), raw_dq(7, 0.08);
    constexpr std::int64_t start_ns = 1'000'000'000;
    single_arm_rt::Arm2StateGuard guard(baseline, start_ns);
    Require(!guard.Ready(), "one snapshot must not satisfy the stationary window");
    for (int i = 0; i < 200; ++i) guard.Check(baseline, raw_dq, start_ns);
    Require(!guard.Ready(), "duplicate timestamps must not warm the stationary window");
    guard.Check(baseline, raw_dq, start_ns + 50'000'000);
    guard.Check(baseline, raw_dq, start_ns + 99'999'999);
    Require(!guard.Ready(), "less than 100 ms must not satisfy the stationary window");
    guard.Check(baseline, raw_dq, start_ns + 100'000'000);
    Require(guard.Ready(), "100 ms of fresh position samples must satisfy the window");
}

void TestArm2WindowIgnoresNoisyRawDq() {
    const std::vector<double> baseline(7, 0.0);
    constexpr std::int64_t start_ns = 1'000'000'000;
    single_arm_rt::Arm2StateGuard guard(baseline, start_ns);
    for (int sample = 1; sample <= 2000; ++sample) {
        std::vector<double> q = baseline, raw_dq(7, 0.0);
        for (std::size_t joint = 0; joint < q.size(); ++joint) {
            q[joint] += 3e-6 * std::sin(sample * 0.17 + joint);
            raw_dq[joint] = 0.08 * std::cos(sample * 0.31 + joint);
        }
        guard.Check(q, raw_dq, start_ns + sample * 1'000'000LL);
        Require(guard.Ready() == (sample >= 100),
                "1 kHz ring-buffer wrap must preserve the stationary window");
    }
}

void TestArm2WindowRecordedStationarySamples() {
    struct RecordedState {
        std::int64_t timestamp_ns;
        std::vector<double> q, raw_dq;
    };
    // User-provided read-only snapshots: position barely changes while the
    // unfiltered link-encoder velocity reaches 0.08276 rad/s. Interpolate at
    // 1 kHz to replay these sparse observations at the controller sample rate.
    const std::array<RecordedState, 3> recorded {{
        {1'792'571'347'856'074'630LL,
         {-8.30945009511197e-06, -0.6985044479370117, 3.937211204174673e-06,
          1.57083261013031, 9.100048919208348e-05, -0.8727245926856995, -5.086872261017561e-05},
         {0.037192050367593765, -0.04909459874033928, 0.026556346565485,
          0.010198217816650867, -0.02098766528069973, 0.08275847136974335, 0.04612991213798523}},
        {1'792'571'348'857'071'718LL,
         {-1.1175317013112362e-05, -0.6984991431236267, 5.03823503095191e-06,
          1.57083261013031, 9.583742212271318e-05, -0.8727251887321472, -4.72789652121719e-05},
         {-0.012137537822127342, -0.01755288802087307, 0.007465977221727371,
          0.015820147469639778, -0.032119616866111755, 0.018109537661075592, -0.04708496481180191}},
        {1'792'571'349'858'067'467LL,
         {-1.2287937352084555e-05, -0.6985021829605103, 5.337307356967358e-06,
          1.570829153060913, 9.230216528521851e-05, -0.8727258443832397, -4.6872362872818485e-05},
         {0.033599697053432465, 0.027626503258943558, 0.023968463763594627,
          -0.026028109714388847, 0.014245637692511082, 0.07565292716026306, 0.0018798772944137454}},
    }};
    single_arm_rt::Arm2StateGuard guard(recorded[0].q, recorded[0].timestamp_ns);
    guard.Check(recorded[0].q, recorded[0].raw_dq, recorded[0].timestamp_ns);
    for (std::size_t segment = 1; segment < recorded.size(); ++segment) {
        const auto& first = recorded[segment - 1];
        const auto& last = recorded[segment];
        for (auto timestamp_ns = first.timestamp_ns + 1'000'000;
             timestamp_ns < last.timestamp_ns; timestamp_ns += 1'000'000) {
            const double fraction = static_cast<double>(timestamp_ns - first.timestamp_ns)
                / static_cast<double>(last.timestamp_ns - first.timestamp_ns);
            auto q = first.q, raw_dq = first.raw_dq;
            for (std::size_t joint = 0; joint < q.size(); ++joint) {
                q[joint] += fraction * (last.q[joint] - first.q[joint]);
                raw_dq[joint] += fraction * (last.raw_dq[joint] - first.raw_dq[joint]);
            }
            guard.Check(q, raw_dq, timestamp_ns);
        }
        guard.Check(last.q, last.raw_dq, last.timestamp_ns);
        Require(guard.Ready(), "recorded stationary positions must pass despite raw dq noise");
    }
}

void TestArm2WindowImmediateDriftAndReversal() {
    const std::vector<double> baseline(7, 0.0), raw_dq(7, 0.0);
    constexpr std::int64_t start_ns = 1'000'000'000;
    single_arm_rt::Arm2StateGuard startup(baseline, start_ns);
    auto q = baseline;
    q[2] = 0.0011;
    Rejects([&] { startup.Check(q, raw_dq, start_ns + 1'000'000); },
            "baseline displacement must stop immediately before speed-window warmup");
    single_arm_rt::Arm2StateGuard duplicate(baseline, start_ns);
    Rejects([&] { duplicate.Check(q, raw_dq, start_ns); },
            "duplicate timestamps must not bypass baseline displacement checks");

    single_arm_rt::Arm2StateGuard reversal(baseline, start_ns);
    for (int sample = 1; sample <= 150; ++sample) {
        q = baseline;
        q[4] = -0.0009 * sample / 150.0;
        reversal.Check(q, raw_dq, start_ns + sample * 1'000'000LL);
    }
    bool stopped = false;
    for (int sample = 151; sample <= 250; ++sample) {
        q[4] = -0.0009 + 0.0018 * (sample - 150) / 100.0;
        Require(std::abs(q[4]) < 0.001, "reversal fixture exceeded baseline drift limit");
        try {
            reversal.Check(q, raw_dq, start_ns + sample * 1'000'000LL, "real-time motion");
        } catch (const std::runtime_error& error) {
            const std::string message = error.what();
            Require(message.find("speed_100ms=") != std::string::npos
                    && message.find("speed_source=position_window_100ms") != std::string::npos,
                    "reversal must stop using measured position-window speed");
            stopped = true;
            break;
        }
    }
    Require(stopped, "fast reversal must stop even while all positions remain inside baseline limit");
}

void TestArm2WindowInvalidStatesAndTimestamps() {
    const std::vector<double> baseline(7, 0.0), raw_dq(7, 0.0);
    constexpr std::int64_t start_ns = 1'000'000'000;
    const std::vector<double> short_state(6, 0.0);
    Rejects([&] { (void)single_arm_rt::Arm2StateGuard(short_state, start_ns); },
            "position-window constructor must require seven baseline joints");
    auto invalid = baseline;
    invalid[1] = std::numeric_limits<double>::quiet_NaN();
    Rejects([&] { (void)single_arm_rt::Arm2StateGuard(invalid, start_ns); },
            "position-window constructor must reject nonfinite baseline");
    Rejects([&] { (void)single_arm_rt::Arm2StateGuard(baseline, -1); },
            "position-window constructor must reject negative timestamp");

    single_arm_rt::Arm2StateGuard guard(baseline, start_ns);
    Rejects([&] { guard.Check(short_state, raw_dq, start_ns); },
            "duplicate timestamps must not bypass position shape checks");
    Rejects([&] { guard.Check(baseline, short_state, start_ns); },
            "duplicate timestamps must not bypass raw dq shape checks");
    Rejects([&] { guard.Check(invalid, raw_dq, start_ns); },
            "duplicate timestamps must not bypass finite position checks");
    Rejects([&] { guard.Check(baseline, invalid, start_ns); },
            "raw dq must remain finite even though it is not a speed limit input");
    Rejects([&] { guard.Check(baseline, raw_dq, -1); },
            "negative timestamps must be rejected");
    Rejects([&] { guard.Check(baseline, raw_dq, start_ns - 1); },
            "regressed timestamps must be rejected");
    Require(!guard.Ready(), "invalid snapshots must not warm the stationary window");
}
}  // namespace

int main() {
    const std::vector<std::pair<std::string, std::function<void()>>> tests{
        {"endpoints and tool-relative tilt", TestEndpointsAndToolRelativeTilt},
        {"measured Home quaternion and relative 15-degree tilt", TestHomeQuaternionStartsAtMeasuredOrientation},
        {"captured start position/orientation and quaternion-sign guard", TestCapturedStartPoseGuard},
        {"numerical velocity and acceleration limits", TestNumericalVelocityAndAccelerationLimits},
        {"bounds and safety margin", TestBoundsAndMargin},
        {"invalid and stationary inputs", TestInvalidAndStationaryInputs},
        {"target convergence", TestReachedUsesPositionAndOrientation},
        {"ARM_2 displacement and speed guard", TestArm2Guard},
        {"ARM_2 measured drift and speed diagnostics", TestArm2GuardDiagnostics},
        {"ARM_2 exact stationary thresholds", TestArm2GuardExactLimits},
        {"ARM_2 position-window warmup and duplicate timestamps", TestArm2WindowWarmupAndDuplicateTimestamps},
        {"ARM_2 1 kHz noisy raw velocity", TestArm2WindowIgnoresNoisyRawDq},
        {"ARM_2 recorded stationary snapshots", TestArm2WindowRecordedStationarySamples},
        {"ARM_2 immediate drift and speed reversal", TestArm2WindowImmediateDriftAndReversal},
        {"ARM_2 position-window invalid states and timestamps", TestArm2WindowInvalidStatesAndTimestamps},
    };
    int failed = 0;
    for (const auto& [name, test] : tests) {
        try {
            test();
            std::cout << "PASS: " << name << '\n';
        } catch (const std::exception& exception) {
            ++failed;
            std::cerr << "FAIL: " << name << ": " << exception.what() << '\n';
        }
    }
    return failed ? 1 : 0;
}
