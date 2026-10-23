/**
 * Modified from Flexiv intermediate6_realtime_cartesian_motion_force_control.cpp.
 *
 * All Home, primitive, Cartesian targets and per-arm settings address ARM_1 only.
 * ARM_1 must exist; other available arm groups are never added to command maps.
 * ServoOn, mode switching and Stop are robot-wide SDK operations. ServoOn requires
 * explicit --auto-servo-on; otherwise the robot must already be operational.
 *
 * 1. Home ARM_1, then tilt TCP orientation 15 degrees around WORLD +Y in free space
 *    (relative to the orientation measured after Home()).
 * 2. Zero the force/torque sensor with the TCP already tilted.
 * 3. Move 50 mm along the tilted TCP +Z axis using position control; wait for arrival.
 * 4. Press toward TCP +Z with 5 N for 6 seconds while retaining the tilt.
 *    With --polish, also execute a smooth 5 cm WORLD +Y line in 5 seconds.
 *
 * IMPORTANT: This is an example for supervised validation, NOT a validated
 * production safety controller. Check workspace, tool TCP/payload, workpiece,
 * force sign, descent distance, and collision clearance before any real motion.
 */

#include <flexiv/rdk/robot.hpp>
#include <flexiv/rdk/scheduler.hpp>
#include <flexiv/rdk/utility.hpp>
#include <spdlog/spdlog.h>
#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>

using namespace flexiv;

namespace {
constexpr double kLoopPeriod = 0.001;
constexpr double kPi = 3.14159265358979323846;
constexpr auto kControlledArm = rdk::JointGroup::ARM_1;
constexpr auto kForceControlFrame = rdk::CoordType::TCP;
enum class CartesianMethod { NRT, MOVEL };
using Pose = std::array<double, rdk::kPoseSize>;

struct Diagnostics {
    const char* phase = "startup";
    bool has_tilt = false;
    Pose start {}, target {};
    std::vector<double> start_q;
    bool has_descent = false;
    Pose descent_start {}, descent_target {};
    std::vector<double> descent_q;
};

template <class Values>
std::string PreciseValues(const Values& values)
{
    std::ostringstream output;
    output << std::setprecision(17);
    for (const auto value : values) output << value << ' ';
    return output.str();
}

void ReportFailure(rdk::Robot& robot, const Diagnostics& diagnostic, bool stop_returned) noexcept
{
    try {
        spdlog::error("CONTROL_FAILED phase={} stop_request_returned={}",
            diagnostic.phase, stop_returned);
        if (diagnostic.has_tilt) {
            spdlog::error("TILT_START xyz_qw_qx_qy_qz={}", PreciseValues(diagnostic.start));
            spdlog::error("TILT_TARGET xyz_qw_qx_qy_qz={}", PreciseValues(diagnostic.target));
            spdlog::error("TILT_START_ARM_1_Q_RAD={}", PreciseValues(diagnostic.start_q));
        }
        if (diagnostic.has_descent) {
            spdlog::error("TCP_Z_MOVE_START xyz_qw_qx_qy_qz={}", PreciseValues(diagnostic.descent_start));
            spdlog::error("TCP_Z_MOVE_TARGET xyz_qw_qx_qy_qz={}", PreciseValues(diagnostic.descent_target));
            spdlog::error("TCP_Z_MOVE_START_ARM_1_Q_RAD={}", PreciseValues(diagnostic.descent_q));
        }
        spdlog::error("FAULT_STATE fault={} status={} mode={} all_stopped={}",
            robot.fault(), static_cast<int>(robot.operational_status()),
            static_cast<int>(robot.mode()), robot.all_stopped());
        const auto states = robot.states();
        const auto arm1 = states.find(kControlledArm);
        if (arm1 != states.end()) {
            spdlog::error("FAULT_ARM_1_POSE={}", PreciseValues(arm1->second.tcp_pose));
            spdlog::error("FAULT_ARM_1_Q_RAD={}", PreciseValues(arm1->second.q));
        }
    } catch (const std::exception& e) {
        spdlog::error("Failure state snapshot unavailable: {}", e.what());
    }
    try {
        // The fault flag may arrive before the asynchronous event details. Stop was
        // already attempted; give an empty queue up to 200 ms to receive the event.
        auto events = robot.event_log();
        for (int attempt = 0; events.empty() && attempt < 10; ++attempt) {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            events = robot.event_log();
        }
        for (const auto& event : events) {
            spdlog::error("ROBOT_EVENT id={} description={} causes={} actions={}",
                event.id, event.description, event.probable_causes, event.recommended_actions);
        }
        if (events.empty()) spdlog::warn("No controller event details received within 200 ms");
    } catch (const std::exception& e) {
        spdlog::error("Robot events unavailable: {}", e.what());
    }
}

// The tilt is relative to the initial TCP orientation and applied around WORLD Y.
constexpr double kTiltAngleDegrees = 15.0;

// Straight path: commanded TCP position changes only along WORLD +Y.
constexpr double kLineDistance = 0.05;   // m
constexpr double kLineDuration = 5.0;    // s
constexpr double kHoldAfterLine = 1.0;  // s

constexpr double kPressingForce = 5.0; // N; sensed TCP Fz=-5 causes pressing toward TCP +Z.
constexpr double kDescentVelocity = 0.005; // m/s
constexpr double kDescentDistance = 0.050; // m; full TCP +Z displacement after tilt/zero
constexpr double kDescentPositionTolerance = 0.0005; // m; measured arrival tolerance
constexpr double kDescentAngularTolerance = 0.02; // rad; keep the tilted orientation

const std::array<double, rdk::kCartDoF> kMaxWrenchForDescent
    = {10.0, 10.0, 10.0, 3.0, 3.0, 3.0};

std::atomic<bool> g_stop_sched {false};
volatile std::sig_atomic_t g_sigint_requested = 0;

void OnSigInt(int) { g_sigint_requested = 1; }

double Smoothstep5(double u)
{
    u = std::clamp(u, 0.0, 1.0);
    return u * u * u * (10.0 + u * (-15.0 + 6.0 * u));
}

Eigen::Quaterniond PoseQuaternion(const std::array<double, rdk::kPoseSize>& pose)
{
    for (const double value : pose) {
        if (!std::isfinite(value)) throw std::invalid_argument("Non-finite TCP pose");
    }
    // Flexiv ordering: [x, y, z, qw, qx, qy, qz].
    const Eigen::Quaterniond rotation(pose[3], pose[4], pose[5], pose[6]);
    if (!std::isfinite(rotation.norm()) || rotation.norm() < 1e-9) {
        throw std::invalid_argument("Invalid TCP quaternion norm");
    }
    return rotation.normalized();
}

Pose AdvanceAlongTcpPositiveZ(const Pose& start_pose)
{
    const auto rotation = PoseQuaternion(start_pose);
    // Freeze the measured post-tilt TCP axis once; Cartesian SDK targets use WORLD coordinates.
    const Eigen::Vector3d delta_world = rotation * Eigen::Vector3d(0.0, 0.0, kDescentDistance);
    auto target = start_pose;
    for (size_t i = 0; i < 3; ++i) target[i] += delta_world[i];
    target[3] = rotation.w();
    target[4] = rotation.x();
    target[5] = rotation.y();
    target[6] = rotation.z();
    return target;
}

std::array<double, rdk::kPoseSize> TiltPose15DegWorldY(
    const std::array<double, rdk::kPoseSize>& start_pose)
{
    auto result = start_pose;
    const Eigen::Quaterniond q0 = PoseQuaternion(start_pose);
    const Eigen::Quaterniond q_delta(Eigen::AngleAxisd(
        kTiltAngleDegrees * kPi / 180.0, Eigen::Vector3d::UnitY()));
    // Left multiply: rotate around an axis FIXED in the WORLD frame.
    const Eigen::Quaterniond q_goal = (q_delta * q0).normalized();
    result[3] = q_goal.w();
    result[4] = q_goal.x();
    result[5] = q_goal.y();
    result[6] = q_goal.z();
    return result;
}

rdk::PrimitiveArgs MoveLCommand(
    const Pose& pose, const std::vector<double>& reference_q, double linear_velocity)
{
    const auto rotation = PoseQuaternion(pose);
    const auto rpy_deg = rdk::utility::Rad2Deg(rdk::utility::Quat2EulerZYX(
        {rotation.w(), rotation.x(), rotation.y(), rotation.z()}));
    if (reference_q.size() != rdk::kSerialJointDoF) {
        throw std::invalid_argument("MoveL requires seven measured ARM_1 reference joints");
    }
    std::array<double, rdk::kSerialJointDoF> reference_deg {};
    for (size_t i = 0; i < reference_deg.size(); ++i) {
        if (!std::isfinite(reference_q[i])) throw std::invalid_argument("Non-finite ARM_1 joint");
        reference_deg[i] = rdk::utility::Rad2Deg(reference_q[i]);
    }
    // Coord orientation and MoveL angVel use degrees; direct NRT commands use radians.
    const rdk::Coord target({pose[0], pose[1], pose[2]}, rpy_deg,
        {"WORLD", "WORLD_ORIGIN"}, reference_deg);
    rdk::PrimitiveArgs command("MoveL", {{"target", target}, {"vel", linear_velocity},
        {"angVel", rdk::utility::Rad2Deg(0.10)}, {"acc", 0.10},
        {"zoneRadius", std::string("ZFine")}});
    command.external_axis_control = false;
    command.sync_motion_mode = rdk::SyncMotionMode::DISABLE;
    return command;
}

} // namespace

void PrintHelp()
{
    std::cout << "Required: [robot_sn]\n"
              << "Optional: [--arm ARM_1] [--TCP] [--polish] [--auto-servo-on]\n"
              << "          [--tilt-method nrt|movel] [--tilt-only]\n"
              << "          [--descent-method nrt|movel]\n"
              << "  All motion targets, including startup Home, address ARM_1 only.\n"
              << "  --arm: only ARM_1 is accepted; this is also the default.\n"
              << "  Tilt: +15 deg about WORLD Y relative to the post-Home orientation.\n"
              << "  Move: tilted TCP +Z by 50.0 mm at 5.0 mm/s, then wait for arrival.\n"
              << "        p_goal = p_start + R_world_tcp(start) * [0, 0, 0.050] m.\n"
              << "        Position tolerance: 0.5 mm; force does not end this move early.\n"
              << "  Press: TCP +Z, 5.0 N (SDK sensed-force target TCP Fz=-5 N).\n"
              << "  Force control is fixed to TCP; --TCP explicitly selects the default.\n"
              << "  --polish: execute the 5 cm WORLD +Y line after 15-degree tilt;\n"
              << "            otherwise hold the motion-controlled axes and orientation for 6 s.\n"
              << "  Startup includes ARM_1 Home, tilt, F/T zeroing and the 50 mm move.\n"
              << "  --tilt-method: initial tilt planner, default nrt; movel uses MoveL.\n"
              << "  --descent-method: independent 50 mm descent planner, default nrt.\n"
              << "                    movel sends one MoveL; no automatic fallback/retry.\n"
              << "  --tilt-only: Home ARM_1, tilt, Stop and exit before zero/descent/RT.\n"
              << "  --auto-servo-on: permit robot-wide ServoOn if not enabled;\n"
              << "                   this may release both arms' brakes.\n"
              << "  Without --auto-servo-on the robot must already be operational.\n";
}

void PeriodicTask(
    rdk::Robot& robot,
    const std::array<double, rdk::kPoseSize>& arm1_init_pose,
    bool enable_polish)
{
    static uint64_t loop_counter = 0;
    try {
        if (robot.fault()) {
            throw std::runtime_error("Robot fault during RT control");
        }

        const double t = loop_counter * kLoopPeriod;
        // RDK commands the reaction sensed at the TCP: -TCP Fz presses toward TCP +Z.
        const std::array<double, rdk::kCartDoF> target_wrench
            = {0.0, 0.0, -kPressingForce, 0.0, 0.0, 0.0};

        auto target_pose = arm1_init_pose;
        if (enable_polish) {
            target_pose[1] = arm1_init_pose[1]
                + kLineDistance * Smoothstep5(t / kLineDuration);
        }
        // Orientation stays fixed; local TCP Z is force-controlled. The force controller
        // may adjust WORLD X/Y/Z along that tilted axis to track the pressing force.
        const std::map<rdk::JointGroup, rdk::RtCartesianCmd> rt_cmds {
            {kControlledArm, rdk::RtCartesianCmd(target_pose, target_wrench)}};

        robot.StreamCartesianMotionForce(rt_cmds);
        ++loop_counter;

        // Stream the final pose briefly, then stop rather than running forever.
        if (t >= kLineDuration + kHoldAfterLine) {
            g_stop_sched = true;
        }
    } catch (const std::exception& e) {
        spdlog::error("PeriodicTask: {}", e.what());
        g_stop_sched = true;
    }
}

int main(int argc, char* argv[])
{
    if (argc < 2 || rdk::utility::ProgramArgsExistAny(argc, argv, {"-h", "--help"})) {
        PrintHelp();
        return argc < 2 ? 1 : 0;
    }

    const std::string robot_sn = argv[1];
    try {
        bool enable_polish = false;
        bool auto_servo_on = false;
        bool tilt_only = false;
        auto tilt_method = CartesianMethod::NRT;
        auto descent_method = CartesianMethod::NRT;
        for (int i = 2; i < argc; ++i) {
            const std::string argument = argv[i];
            if (argument == "--polish") {
                enable_polish = true;
            } else if (argument == "--TCP") {
                // TCP force control is already the fixed default for this workflow.
            } else if (argument == "--auto-servo-on") {
                auto_servo_on = true;
            } else if (argument == "--tilt-only") {
                tilt_only = true;
            } else if (argument == "--tilt-method") {
                if (++i >= argc) throw std::invalid_argument("Missing --tilt-method value");
                const std::string method = argv[i];
                if (method == "nrt") tilt_method = CartesianMethod::NRT;
                else if (method == "movel") tilt_method = CartesianMethod::MOVEL;
                else throw std::invalid_argument("--tilt-method accepts nrt or movel");
            } else if (argument == "--descent-method") {
                if (++i >= argc) throw std::invalid_argument("Missing --descent-method value");
                const std::string method = argv[i];
                if (method == "nrt") descent_method = CartesianMethod::NRT;
                else if (method == "movel") descent_method = CartesianMethod::MOVEL;
                else throw std::invalid_argument("--descent-method accepts nrt or movel");
            } else if (argument == "--arm") {
                if (++i >= argc || std::string(argv[i]) != "ARM_1") {
                    throw std::invalid_argument("Only --arm ARM_1 is supported");
                }
            } else {
                throw std::invalid_argument("Unknown argument: " + argument);
            }
        }
        std::signal(SIGINT, OnSigInt);
        rdk::Robot robot(robot_sn);
        // Validate before any control request, including ServoOn, Home or Stop.
        const auto available_groups = robot.info().single_arm_groups;
        if (!available_groups.contains(kControlledArm)) {
            throw std::runtime_error("ARM_1 is not available on this robot");
        }
        const std::map<rdk::JointGroup, std::string> controlled_groups {
            {kControlledArm, available_groups.at(kControlledArm)}};
        if (robot.fault()) {
            throw std::runtime_error("Clear the robot fault before starting ARM_1 control");
        }
        if (robot.mode() != rdk::Mode::IDLE || !robot.all_stopped()) {
            throw std::runtime_error("Robot must be IDLE and all-stopped before starting");
        }
        const bool needs_servo_on = !robot.operational();
        if (needs_servo_on && (!auto_servo_on
                || robot.operational_status() != rdk::OperationalStatus::NOT_SERVO_ON)) {
            throw std::runtime_error(
                "Robot is not ready; --auto-servo-on permits robot-wide brake release only from NOT_SERVO_ON");
        }
        Diagnostics diagnostic;
        try {
        if (g_sigint_requested) throw std::runtime_error("User interrupted");
        if (needs_servo_on) {
            diagnostic.phase = "servo_on";
            spdlog::warn("Explicit ServoOn: this may release both arms' brakes");
            robot.ServoOn();
            const auto ready_deadline
                = std::chrono::steady_clock::now() + std::chrono::seconds(10);
            while (!robot.operational()) {
                if (robot.fault() || g_sigint_requested
                    || std::chrono::steady_clock::now() > ready_deadline) {
                    throw std::runtime_error("ServoOn interrupted/timeout/fault");
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
            }
        }
        if (g_sigint_requested) throw std::runtime_error("User interrupted");
        spdlog::info("ARM_1 only: Home; tilt WORLD +Y {:.1f} deg; move TCP +Z {:.1f} mm; press TCP +Z {:.1f} N",
            kTiltAngleDegrees, kDescentDistance * 1000.0, kPressingForce);
        diagnostic.phase = "home";
        robot.Home({kControlledArm});

        // Phase 1: tilt in FREE SPACE, before approaching the workpiece.
        diagnostic.phase = "tilt_plan";
        const auto after_home = robot.states().at(kControlledArm);
        diagnostic.start = after_home.tcp_pose;
        diagnostic.start_q = after_home.q;
        diagnostic.target = TiltPose15DegWorldY(diagnostic.start);
        diagnostic.has_tilt = true;
        const double tilt_degrees = rdk::utility::Rad2Deg(
            PoseQuaternion(diagnostic.start).angularDistance(PoseQuaternion(diagnostic.target)));
        spdlog::info("TILT_PLAN method={} delta_deg={:.9f} position_delta_m=0",
            tilt_method == CartesianMethod::MOVEL ? "movel" : "nrt", tilt_degrees);
        spdlog::info("TILT_START xyz_qw_qx_qy_qz={}", PreciseValues(diagnostic.start));
        spdlog::info("TILT_TARGET xyz_qw_qx_qy_qz={}", PreciseValues(diagnostic.target));
        spdlog::info("TILT_START_ARM_1_Q_RAD={}", PreciseValues(diagnostic.start_q));
        std::map<rdk::JointGroup, std::array<double, rdk::kPoseSize>> tilted_target_poses;
        tilted_target_poses[kControlledArm] = diagnostic.target;
        if (tilt_method == CartesianMethod::MOVEL) {
            const auto command = MoveLCommand(diagnostic.target, after_home.q, 0.01);
            diagnostic.phase = "tilt_movel_submit";
            robot.SwitchMode(rdk::Mode::NRT_PRIMITIVE_EXECUTION);
            robot.ExecutePrimitive({{kControlledArm, command}});
        } else {
            diagnostic.phase = "tilt_nrt_submit";
            robot.SwitchMode(rdk::Mode::NRT_CARTESIAN_MOTION_FORCE);
            robot.SendCartesianMotionForce({{kControlledArm, rdk::NrtCartesianCmd(
                diagnostic.target, {}, {}, 0.01, 0.10, 0.05, 0.30)}});
        }
        diagnostic.phase = tilt_method == CartesianMethod::MOVEL ? "tilt_movel_wait" : "tilt_nrt_wait";

        const auto tilt_deadline
            = std::chrono::steady_clock::now() + std::chrono::seconds(15);
        while (true) {
            if (robot.fault() || g_sigint_requested) {
                throw std::runtime_error("Tilt interrupted or robot fault");
            }
            const auto states = robot.states();
            bool all_reached = true;
            for (const auto& [group, goal_pose] : tilted_target_poses) {
                const auto& current = states.at(group).tcp_pose;
                const Eigen::Vector3d p_now(current[0], current[1], current[2]);
                const Eigen::Vector3d p_goal(goal_pose[0], goal_pose[1], goal_pose[2]);
                const double pos_error = (p_now - p_goal).norm();
                const double ang_error
                    = PoseQuaternion(current).angularDistance(PoseQuaternion(goal_pose));
                if (pos_error > 0.004 || ang_error > 0.02) {
                    all_reached = false;
                }
            }
            if (tilt_method == CartesianMethod::MOVEL) {
                const auto primitive_states = robot.primitive_states();
                const auto arm1 = primitive_states.find(kControlledArm);
                const bool correct_primitive = arm1 != primitive_states.end()
                    && arm1->second.pt_name == "MoveL";
                const bool reached = correct_primitive
                    && rdk::utility::PrimitiveStateTrue(arm1->second, "reachedTarget");
                if (correct_primitive && !reached
                    && rdk::utility::PrimitiveStateTrue(arm1->second, "terminated")) {
                    throw std::runtime_error("ARM_1 MoveL terminated without reachedTarget");
                }
                all_reached = all_reached && reached;
            }
            if (all_reached) break;
            if (std::chrono::steady_clock::now() > tilt_deadline) {
                throw std::runtime_error("Tilt pose not reached before timeout");
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        spdlog::info("Tilt reached. Keep the arm clear of all objects.");
        if (tilt_only) {
            diagnostic.phase = "tilt_only_stop";
            robot.Stop();
            spdlog::info("ARM_1 tilt-only complete; no F/T zeroing, Z descent or RT pressing started");
            return 0;
        }

        // Phase 2: zero the F/T sensor at the FINAL tilted orientation.
        diagnostic.phase = "zero_ft";
        robot.SwitchMode(rdk::Mode::NRT_PRIMITIVE_EXECUTION);
        std::map<rdk::JointGroup, rdk::PrimitiveArgs> pt_args;
        for (const auto& [group, _] : controlled_groups) {
            pt_args[group] = rdk::PrimitiveArgs("ZeroFTSensor", {});
            pt_args[group].external_axis_control = false;
            pt_args[group].sync_motion_mode = rdk::SyncMotionMode::DISABLE;
        }
        spdlog::warn("ZeroFTSensor: robot must NOT touch any object");
        robot.ExecutePrimitive(pt_args);
        const auto zero_deadline
            = std::chrono::steady_clock::now() + std::chrono::seconds(30);
        while (true) {
            if (robot.fault() || g_sigint_requested
                || std::chrono::steady_clock::now() > zero_deadline) {
                throw std::runtime_error("ZeroFTSensor interrupted/timeout/fault");
            }
            const auto primitive_states = robot.primitive_states();
            const auto arm1_state = primitive_states.find(kControlledArm);
            if (arm1_state != primitive_states.end()
                && arm1_state->second.pt_name == "ZeroFTSensor"
                && rdk::utility::PrimitiveStateTrue(arm1_state->second, "terminated")) {
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        // Phase 3: complete the fixed, post-tilt TCP +Z position move before force control.
        diagnostic.phase = "tcp_z_descent";
        std::map<rdk::JointGroup, std::array<double, rdk::kPoseSize>> all_init_pose;
        const auto states_after_zero = robot.states();
        for (const auto& [group, _] : controlled_groups) {
            all_init_pose[group] = states_after_zero.at(group).tcp_pose;
        }
        std::map<rdk::JointGroup, rdk::NrtCartesianCmd> descent_cmds;
        for (const auto& [group, start_pose] : all_init_pose) {
            const auto descent_pose = AdvanceAlongTcpPositiveZ(start_pose);
            descent_cmds[group] = rdk::NrtCartesianCmd(
                descent_pose, {}, {}, kDescentVelocity);
            spdlog::info("TCP_Z_MOVE_START xyz_qw_qx_qy_qz={}", PreciseValues(start_pose));
            spdlog::info("TCP_Z_MOVE_TARGET xyz_qw_qx_qy_qz={}", PreciseValues(descent_pose));
            const std::array<double, 3> world_delta = {descent_pose[0] - start_pose[0],
                descent_pose[1] - start_pose[1], descent_pose[2] - start_pose[2]};
            spdlog::info("TCP_Z_MOVE_WORLD_DELTA_M={}", PreciseValues(world_delta));
        }
        diagnostic.descent_start = all_init_pose.at(kControlledArm);
        diagnostic.descent_target = descent_cmds.at(kControlledArm).pose_d;
        diagnostic.descent_q = states_after_zero.at(kControlledArm).q;
        diagnostic.has_descent = true;
        if (descent_method == CartesianMethod::MOVEL) {
            const auto command = MoveLCommand(
                diagnostic.descent_target, diagnostic.descent_q, kDescentVelocity);
            diagnostic.phase = "tcp_z_descent_movel_submit";
            robot.SwitchMode(rdk::Mode::NRT_PRIMITIVE_EXECUTION);
            robot.ExecutePrimitive({{kControlledArm, command}});
        } else {
            diagnostic.phase = "tcp_z_descent_nrt_submit";
            robot.SwitchMode(rdk::Mode::NRT_CARTESIAN_MOTION_FORCE);
            robot.SetMaxContactWrench(kControlledArm, kMaxWrenchForDescent);
            robot.SetForceControlAxis(kControlledArm, {});
            robot.SendCartesianMotionForce(descent_cmds);
        }
        diagnostic.phase = descent_method == CartesianMethod::MOVEL
            ? "tcp_z_descent_movel_wait" : "tcp_z_descent_nrt_wait";
        spdlog::info("Moving TCP +Z by {:.1f} mm at {:.1f} mm/s with {}; waiting for position arrival and ARM_1 stop",
            kDescentDistance * 1000.0, kDescentVelocity * 1000.0,
            descent_method == CartesianMethod::MOVEL ? "MoveL" : "direct NRT");

        const auto descent_deadline
            = std::chrono::steady_clock::now() + std::chrono::seconds(20);
        while (true) {
            if (robot.fault() || g_sigint_requested) {
                throw std::runtime_error("TCP +Z position move interrupted or robot fault; RT pressing was not started");
            }
            const auto states = robot.states();
            const auto stopped_groups = robot.stopped();
            bool all_reached = true;
            for (const auto& [group, _] : controlled_groups) {
                const auto& measured = states.at(group);
                const auto& current = measured.tcp_pose;
                const auto& target = descent_cmds.at(group).pose_d;
                const auto current_rotation = PoseQuaternion(current);
                // MoveL does not expose SetMaxContactWrench in primitive mode.
                // Monitor the same finite component limits on the host for both methods.
                for (size_t axis = 0; axis < measured.tcp_wrench.size(); ++axis) {
                    if (!std::isfinite(measured.tcp_wrench[axis])) {
                        throw std::runtime_error("Non-finite WORLD wrench during descent");
                    }
                    if (std::abs(measured.tcp_wrench[axis]) > kMaxWrenchForDescent[axis]) {
                        throw std::runtime_error("WORLD contact-wrench limit exceeded during descent");
                    }
                }
                const Eigen::Vector3d current_position(current[0], current[1], current[2]);
                const Eigen::Vector3d target_position(target[0], target[1], target[2]);
                const bool arm_stopped = stopped_groups.contains(group) && stopped_groups.at(group);
                if ((current_position - target_position).norm() > kDescentPositionTolerance
                    || current_rotation.angularDistance(PoseQuaternion(target)) > kDescentAngularTolerance
                    || !arm_stopped) {
                    all_reached = false;
                }
            }
            if (descent_method == CartesianMethod::MOVEL) {
                const auto primitive_states = robot.primitive_states();
                const auto arm1 = primitive_states.find(kControlledArm);
                const bool correct_primitive = arm1 != primitive_states.end()
                    && arm1->second.pt_name == "MoveL";
                const bool reached = correct_primitive
                    && rdk::utility::PrimitiveStateTrue(arm1->second, "reachedTarget");
                if (correct_primitive && !reached
                    && rdk::utility::PrimitiveStateTrue(arm1->second, "terminated")) {
                    throw std::runtime_error("ARM_1 descent MoveL terminated without reachedTarget");
                }
                all_reached = all_reached && reached;
            }
            if (all_reached) {
                const auto& measured = states.at(kControlledArm);
                const auto& start_pose = all_init_pose.at(kControlledArm);
                const Eigen::Vector3d displacement_world(measured.tcp_pose[0] - start_pose[0],
                    measured.tcp_pose[1] - start_pose[1], measured.tcp_pose[2] - start_pose[2]);
                const auto displacement_tcp = PoseQuaternion(start_pose).conjugate() * displacement_world;
                spdlog::info("TCP +Z move reached: axial displacement={:.3f} mm, sensed TCP Fz={:.3f} N; starting TCP +Z 5.0 N pressing",
                    displacement_tcp.z() * 1000.0, measured.tcp_wrench_local[2]);
                break;
            }
            if (std::chrono::steady_clock::now() > descent_deadline) {
                throw std::runtime_error("50.0 mm TCP +Z position target not reached before timeout; RT pressing was not started");
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }

        // Phase 4: retain the post-tilt TCP orientation and execute RT motion + local Z force.
        diagnostic.phase = "rt_motion_force";
        robot.SwitchMode(rdk::Mode::RT_CARTESIAN_MOTION_FORCE);
        for (const auto& [group, _] : controlled_groups) {
            robot.SetForceControlFrame(group, kForceControlFrame);
            robot.SetForceControlAxis(group,
                std::array<bool, rdk::kCartDoF>{false, false, true, false, false, false});
        }
        // Reapply finite contact-wrench limits after the control mode transition.
        robot.SetMaxContactWrench(kControlledArm, kMaxWrenchForDescent);

        const auto states_after_descent = robot.states();
        for (auto& [group, pose] : all_init_pose) {
            pose = states_after_descent.at(group).tcp_pose;
            // Keep the same captured orientation used to define the TCP +Z position move.
            const auto& angle_goal = descent_cmds.at(group).pose_d;
            for (size_t i = 3; i < 7; ++i) pose[i] = angle_goal[i];
        }

        g_stop_sched = false;
        rdk::Scheduler scheduler;
        scheduler.AddTask(std::bind(PeriodicTask, std::ref(robot),
                              std::cref(all_init_pose.at(kControlledArm)),
                              enable_polish),
            "HP periodic", 1, scheduler.max_priority());
        scheduler.Start();
        while (!g_stop_sched && !g_sigint_requested) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        scheduler.Stop();
        robot.Stop();
        spdlog::info("Motion terminated; robot switched to IDLE");
        } catch (...) {
            // Exception-safe stop, also for interrupted tilt/descent phases.
            bool stop_returned = false;
            try {
                robot.Stop();
                stop_returned = true;
            } catch (const std::exception& stop_error) {
                spdlog::error("Stop request failed; software has not confirmed a stop: {}",
                    stop_error.what());
            }
            ReportFailure(robot, diagnostic, stop_returned);
            throw;
        }

    } catch (const std::exception& e) {
        spdlog::error("{}", e.what());
        // For production also implement monitored force thresholds,
        // external protective measures, and system-level safety validation.
        return 1;
    }
    return 0;
}
