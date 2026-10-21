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
 * 3. Search for contact along WORLD -Z.
 * 4. Perform a smooth 5 cm WORLD +Y straight-line TCP motion in 5 seconds,
 *    preserving the desired 15-degree tilt while controlling WORLD Z force.
 *
 * IMPORTANT: This is an example for supervised validation, NOT a validated
 * production safety controller. Check workspace, tool TCP/payload, workpiece,
 * force sign, search range, and collision clearance before any real motion.
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
enum class TiltMethod { NRT, MOVEL };
using Pose = std::array<double, rdk::kPoseSize>;

struct Diagnostics {
    const char* phase = "startup";
    bool has_tilt = false;
    Pose start {}, target {};
    std::vector<double> start_q;
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
        // Events are available only for this Robot instance's connection lifetime.
        for (const auto& event : robot.event_log()) {
            spdlog::error("ROBOT_EVENT id={} description={} causes={} actions={}",
                event.id, event.description, event.probable_causes, event.recommended_actions);
        }
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

constexpr double kPressingForce = 5.0; // N; WORLD +Fz causes pressing toward -Z.
constexpr double kContactDetectForce = 3.0; // N
constexpr double kSearchVelocity = 0.005; // m/s
constexpr double kSearchDistance = 0.05;  // m; adjust to real geometry

const std::array<double, rdk::kCartDoF> kMaxWrenchForContactSearch
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
    if (rotation.norm() < 1e-9) throw std::invalid_argument("Invalid zero TCP quaternion");
    return rotation.normalized();
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

rdk::PrimitiveArgs MoveLTiltCommand(const Pose& pose, const std::vector<double>& reference_q)
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
    rdk::PrimitiveArgs command("MoveL", {{"target", target}, {"vel", 0.01},
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
              << "  All motion targets, including startup Home, address ARM_1 only.\n"
              << "  --arm: only ARM_1 is accepted; this is also the default.\n"
              << "  --TCP: force-control along local TCP Z instead of WORLD Z.\n"
              << "  --polish: execute the 5 cm WORLD +Y line after 15-degree tilt;\n"
              << "            otherwise hold position for the same duration.\n"
              << "  Startup includes ARM_1 Home, F/T zeroing and contact search.\n"
              << "  --tilt-method: initial tilt planner, default nrt; movel uses MoveL.\n"
              << "  --tilt-only: Home ARM_1, tilt, Stop and exit before zero/search/RT.\n"
              << "  --auto-servo-on: permit robot-wide ServoOn if not enabled;\n"
              << "                   this may release both arms' brakes.\n"
              << "  Without --auto-servo-on the robot must already be operational.\n";
}

void PeriodicTask(
    rdk::Robot& robot,
    const std::array<double, rdk::kPoseSize>& arm1_init_pose,
    rdk::CoordType force_ctrl_frame,
    bool enable_polish)
{
    static uint64_t loop_counter = 0;
    try {
        if (robot.fault()) {
            throw std::runtime_error("Robot fault during RT control");
        }

        const double t = loop_counter * kLoopPeriod;
        const double Fz = (force_ctrl_frame == rdk::CoordType::WORLD)
            ? kPressingForce : -kPressingForce;
        const std::array<double, rdk::kCartDoF> target_wrench
            = {0.0, 0.0, Fz, 0.0, 0.0, 0.0};

        auto target_pose = arm1_init_pose;
        if (enable_polish) {
            target_pose[1] = arm1_init_pose[1]
                + kLineDistance * Smoothstep5(t / kLineDuration);
        }
        // Orientation stays fixed; Z is force-controlled rather than position-tracked.
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
        auto tilt_method = TiltMethod::NRT;
        auto force_ctrl_frame = rdk::CoordType::WORLD;
        for (int i = 2; i < argc; ++i) {
            const std::string argument = argv[i];
            if (argument == "--polish") {
                enable_polish = true;
            } else if (argument == "--TCP") {
                force_ctrl_frame = rdk::CoordType::TCP;
            } else if (argument == "--auto-servo-on") {
                auto_servo_on = true;
            } else if (argument == "--tilt-only") {
                tilt_only = true;
            } else if (argument == "--tilt-method") {
                if (++i >= argc) throw std::invalid_argument("Missing --tilt-method value");
                const std::string method = argv[i];
                if (method == "nrt") tilt_method = TiltMethod::NRT;
                else if (method == "movel") tilt_method = TiltMethod::MOVEL;
                else throw std::invalid_argument("--tilt-method accepts nrt or movel");
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
        spdlog::info("ARM_1 only: Home, 15-degree tilt, contact search and RT control");
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
            tilt_method == TiltMethod::MOVEL ? "movel" : "nrt", tilt_degrees);
        spdlog::info("TILT_START xyz_qw_qx_qy_qz={}", PreciseValues(diagnostic.start));
        spdlog::info("TILT_TARGET xyz_qw_qx_qy_qz={}", PreciseValues(diagnostic.target));
        spdlog::info("TILT_START_ARM_1_Q_RAD={}", PreciseValues(diagnostic.start_q));
        std::map<rdk::JointGroup, std::array<double, rdk::kPoseSize>> tilted_target_poses;
        tilted_target_poses[kControlledArm] = diagnostic.target;
        if (tilt_method == TiltMethod::MOVEL) {
            const auto command = MoveLTiltCommand(diagnostic.target, after_home.q);
            diagnostic.phase = "tilt_movel_submit";
            robot.SwitchMode(rdk::Mode::NRT_PRIMITIVE_EXECUTION);
            robot.ExecutePrimitive({{kControlledArm, command}});
        } else {
            diagnostic.phase = "tilt_nrt_submit";
            robot.SwitchMode(rdk::Mode::NRT_CARTESIAN_MOTION_FORCE);
            robot.SendCartesianMotionForce({{kControlledArm, rdk::NrtCartesianCmd(
                diagnostic.target, {}, {}, 0.01, 0.10, 0.05, 0.30)}});
        }
        diagnostic.phase = tilt_method == TiltMethod::MOVEL ? "tilt_movel_wait" : "tilt_nrt_wait";

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
            if (tilt_method == TiltMethod::MOVEL) {
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
            spdlog::info("ARM_1 tilt-only complete; no F/T zeroing, contact search or RT line started");
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

        // Phase 3: approach contact with the already tilted TCP orientation.
        diagnostic.phase = "contact_search";
        std::map<rdk::JointGroup, std::array<double, rdk::kPoseSize>> all_init_pose;
        const auto states_after_zero = robot.states();
        for (const auto& [group, _] : controlled_groups) {
            all_init_pose[group] = states_after_zero.at(group).tcp_pose;
        }
        robot.SwitchMode(rdk::Mode::NRT_CARTESIAN_MOTION_FORCE);
        for (const auto& [group, _] : controlled_groups) {
            robot.SetMaxContactWrench(group, kMaxWrenchForContactSearch);
        }
        std::map<rdk::JointGroup, rdk::NrtCartesianCmd> search_cmds;
        for (const auto& [group, start_pose] : all_init_pose) {
            auto search_pose = start_pose;
            search_pose[2] -= kSearchDistance;
            search_cmds[group] = rdk::NrtCartesianCmd(
                search_pose, {}, {}, kSearchVelocity);
        }
        robot.SendCartesianMotionForce(search_cmds);
        spdlog::info("Searching contact, max travel {} m at {} m/s",
            kSearchDistance, kSearchVelocity);

        bool is_contacted = false;
        const auto contact_deadline
            = std::chrono::steady_clock::now() + std::chrono::seconds(20);
        while (!is_contacted) {
            if (robot.fault() || g_sigint_requested
                || std::chrono::steady_clock::now() > contact_deadline) {
                throw std::runtime_error("Contact not found within safe search window");
            }
            const auto states = robot.states();
            for (const auto& [group, _] : controlled_groups) {
                const auto& measured = states.at(group);
                const Eigen::Vector3d ext_force(
                    measured.tcp_wrench[0], measured.tcp_wrench[1], measured.tcp_wrench[2]);
                if (ext_force.norm() > kContactDetectForce) {
                    is_contacted = true;
                    spdlog::info("[{}] Contact detected", rdk::kJointGroupNames.at(group));
                    break;
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }

        // Phase 4: retain tilted quaternion and execute RT motion + Z force.
        diagnostic.phase = "rt_motion_force";
        robot.SwitchMode(rdk::Mode::RT_CARTESIAN_MOTION_FORCE);
        for (const auto& [group, _] : controlled_groups) {
            robot.SetForceControlFrame(group, force_ctrl_frame);
            robot.SetForceControlAxis(group,
                std::array<bool, rdk::kCartDoF>{false, false, true, false, false, false});
        }
        // Deliberately KEEP finite contact-wrench limits instead of disabling them
        // with infinity as in the example. Validate these limits for your setup.

        const auto states_at_contact = robot.states();
        for (auto& [group, pose] : all_init_pose) {
            pose = states_at_contact.at(group).tcp_pose;
            // Force the rotation setpoint to remain EXACTLY the tilt target.
            const auto& angle_goal = tilted_target_poses.at(group);
            for (size_t i = 3; i < 7; ++i) pose[i] = angle_goal[i];
        }

        g_stop_sched = false;
        rdk::Scheduler scheduler;
        scheduler.AddTask(std::bind(PeriodicTask, std::ref(robot),
                              std::cref(all_init_pose.at(kControlledArm)),
                              force_ctrl_frame, enable_polish),
            "HP periodic", 1, scheduler.max_priority());
        scheduler.Start();
        while (!g_stop_sched && !g_sigint_requested) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        scheduler.Stop();
        robot.Stop();
        spdlog::info("Motion terminated; robot switched to IDLE");
        } catch (...) {
            // Exception-safe stop, also for interrupted NRT/contact phases.
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
