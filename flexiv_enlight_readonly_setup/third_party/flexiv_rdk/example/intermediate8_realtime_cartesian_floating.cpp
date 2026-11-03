/**
 * Flexiv Enlight: 6-DoF RT Cartesian floating (RDK v2.x / C++20).
 *
 * Two selectable methods:
 *   (1) default: six Cartesian motion axes with reduced/zero stiffness.
 *   (2) --passive: six Cartesian force axes, passive zero-wrench feedforward.
 *
 * Safety: illustrative software monitoring only, not a safety-rated function.
 * Test in a clear workspace with a trained operator, E-stop and configured
 * hardware safety limits. Do not attach unexpected payloads or allow bystanders.
 */

#include <flexiv/rdk/robot.hpp>
#include <flexiv/rdk/scheduler.hpp>

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <exception>
#include <iostream>
#include <map>
#include <stdexcept>
#include <string>
#include <thread>

namespace rdk = flexiv::rdk;
using Clock = std::chrono::steady_clock;

namespace {

// Conservative EXAMPLE thresholds for first trials; adapt to risk assessment.
constexpr double kMaxTcpLinearSpeed = 0.20;   // m/s
constexpr double kMaxTcpAngularSpeed = 0.70;  // rad/s
constexpr double kMaxTcpDisplacement = 0.20;  // m, relative to start
constexpr double kJointLimitMargin = 0.12;    // rad
constexpr double kJointVelocityFraction = 0.30; // fraction of robot limit
constexpr std::int64_t kHeartbeatTimeoutNs = 100'000'000; // 100 ms

volatile std::sig_atomic_t g_sigint = 0;
void OnSignal(int) { g_sigint = 1; }

enum class StopReason : int {
    None = 0,
    RobotNotReady,
    OverspeedTCP,
    OverspeedJoint,
    WorkspaceExceeded,
    JointLimitNearby,
    StateInvalid,
    StreamFailed,
    HeartbeatLost,
};

void Trip(std::atomic<StopReason>& reason, StopReason why)
{
    auto expected = StopReason::None;
    reason.compare_exchange_strong(expected, why);
}

const char* ReasonText(StopReason r)
{
    switch (r) {
        case StopReason::None: return "No software alarm (operator exit)";
        case StopReason::RobotNotReady: return "Robot fault / connection / operational state";
        case StopReason::OverspeedTCP: return "TCP speed too high";
        case StopReason::OverspeedJoint: return "Joint speed too high";
        case StopReason::WorkspaceExceeded: return "TCP left local test workspace";
        case StopReason::JointLimitNearby: return "Joint position close to limit";
        case StopReason::StateInvalid: return "Unexpected state data / non-finite values";
        case StopReason::StreamFailed: return "RT state reading or streaming failed";
        case StopReason::HeartbeatLost: return "RT callback heartbeat lost";
    }
    return "Unknown";
}

std::int64_t NowNs()
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        Clock::now().time_since_epoch()).count();
}

double Norm3(const std::array<double, rdk::kCartDoF>& v, int offset)
{
    return std::sqrt(v[offset] * v[offset] +
                     v[offset + 1] * v[offset + 1] +
                     v[offset + 2] * v[offset + 2]);
}

void Usage(const char* exe)
{
    std::cerr << "Usage: " << exe
              << " <robot_sn> [--scale=0.10] [--nullspace=0.50] [--passive]\n"
              << "  --scale: fraction of nominal Cartesian stiffness, [0,1].\n"
              << "           0 gives free-floating Cartesian motion axes.\n"
              << "  --nullspace: reference posture tracking weight, [0.1,1].\n"
              << "  --passive: experimental 6-axis passive zero-wrench control.\n";
}

struct Options {
    std::string robot_sn;
    bool passive = false;
    double scale = 0.10;
    double nullspace = 0.50;
};

Options ParseOptions(int argc, char** argv)
{
    if (argc < 2) {
        Usage(argv[0]);
        throw std::invalid_argument("Robot serial number is required");
    }
    Options o;
    o.robot_sn = argv[1];
    for (int i = 2; i < argc; ++i) {
        std::string a(argv[i]);
        if (a == "--passive") {
            o.passive = true;
        } else if (a.rfind("--scale=", 0) == 0) {
            o.scale = std::stod(a.substr(8));
        } else if (a.rfind("--nullspace=", 0) == 0) {
            o.nullspace = std::stod(a.substr(12));
        } else {
            Usage(argv[0]);
            throw std::invalid_argument("Unknown argument: " + a);
        }
    }
    if (!std::isfinite(o.scale) || o.scale < 0 || o.scale > 1) {
        throw std::invalid_argument("--scale must be between 0 and 1");
    }
    if (!std::isfinite(o.nullspace) || o.nullspace < 0.1 || o.nullspace > 1) {
        throw std::invalid_argument("--nullspace must be between 0.1 and 1");
    }
    return o;
}

} // namespace

int main(int argc, char** argv)
{
    std::signal(SIGINT, OnSignal);

    try {
        const Options opts = ParseOptions(argc, argv);
        rdk::Robot robot(opts.robot_sn);
        rdk::Scheduler scheduler;
        bool scheduler_started = false;

        // Always stop streaming before calling Robot::Stop() (blocking).
        auto shutdown = [&]() -> bool {
            bool ok = true;
            if (scheduler_started) {
                scheduler_started = false;
                try {
                    scheduler.Stop();
                } catch (const std::exception& e) {
                    std::cerr << "Scheduler.Stop failed: " << e.what() << '\n';
                    ok = false;
                }
            }
            try {
                robot.Stop();
            } catch (const std::exception& e) {
                std::cerr << "Robot.Stop failed: " << e.what() << '\n';
                ok = false;
            }
            return ok;
        };

        try {
            // Deliberately do NOT automatically clear faults or move to Home.
            if (robot.fault() || !robot.estop_released()) {
                throw std::runtime_error(
                    "Robot has a fault or E-stop is pressed; resolve manually");
            }
            robot.ServoOn();
            const auto op_deadline = Clock::now() + std::chrono::seconds(20);
            while (!robot.operational()) {
                if (g_sigint || Clock::now() > op_deadline) {
                    throw std::runtime_error("Robot did not become operational");
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }

            const auto info = robot.info();
            const auto group = rdk::JointGroup::ARM_1;
            if (!info.single_arm_groups.contains(group)) {
                throw std::runtime_error("Expected ARM_1 joint group not present");
            }

            // Passive-force setup is valid in IDLE only (RDK v2.1).
            if (robot.mode() != rdk::Mode::IDLE) {
                robot.Stop();
            }
            robot.SetPassiveForceControl(group, opts.passive);
            robot.SetTimelinessFailureLimit(2.0); // documented RDK v2.x default
            robot.SwitchMode(rdk::Mode::RT_CARTESIAN_MOTION_FORCE);

            const auto all_initial_states = robot.states();
            const auto& initial = all_initial_states.at(group);
            const auto pose0 = initial.tcp_pose;
            const auto q0 = initial.q;
            const auto q_min = info.q_min.at(group);
            const auto q_max = info.q_max.at(group);
            const auto dq_max = info.dq_max.at(group);

            if (q0.empty() || q0.size() != q_min.size() ||
                q0.size() != q_max.size() || q0.size() != dq_max.size()) {
                throw std::runtime_error("Invalid initial joint limit information");
            }
            for (size_t i = 0; i < q0.size(); ++i) {
                if (!std::isfinite(q0[i]) || !std::isfinite(q_min[i]) ||
                    !std::isfinite(q_max[i]) || !std::isfinite(dq_max[i]) ||
                    dq_max[i] <= 0 ||
                    q0[i] <= q_min[i] + kJointLimitMargin ||
                    q0[i] >= q_max[i] - kJointLimitMargin) {
                    throw std::runtime_error(
                        "Invalid initial joint state or too close to joint limit");
                }
            }
            for (double v : pose0) {
                if (!std::isfinite(v)) {
                    throw std::runtime_error("Initial TCP pose is not finite");
                }
            }

            // Reference null-space posture is the START posture, not arbitrary Home.
            robot.SetNullSpacePosture(group, q0);
            robot.SetNullSpaceObjectives(group, 0.0, 0.0, opts.nullspace);

            // First begin streaming in the nominal pure-motion configuration.
            // Only lower stiffness / enable passive force axes AFTER RT streaming
            // is confirmed, to avoid a no-command window with no pose holding.
            robot.SetForceControlAxis(
                group, {false, false, false, false, false, false});
            robot.SetCartesianImpedance(group, info.K_x_nom.at(group));

            // Stable initial pose. No target motion, twist or acceleration.
            // Zero wrench for passive mode; unused for pure motion control.
            const std::array<double, rdk::kCartDoF> zero_wrench{};
            const std::map<rdk::JointGroup, rdk::RtCartesianCmd> cmds{
                {group, rdk::RtCartesianCmd(pose0, zero_wrench)}
            };

            std::atomic<StopReason> stop_reason{StopReason::None};
            std::atomic<std::int64_t> last_tick_ns{0};

            scheduler.AddTask(
                [&]() {
                    if (stop_reason.load() != StopReason::None) {
                        return;
                    }
                    last_tick_ns.store(NowNs());

                    try {
                        if (!robot.connected() || robot.fault() ||
                            !robot.operational() ||
                            robot.reached_timeliness_failure_limit()) {
                            Trip(stop_reason, StopReason::RobotNotReady);
                            return;
                        }

                        const auto all_states = robot.states();
                        const auto& s = all_states.at(group);

                        const double v_lin = Norm3(s.tcp_twist, 0);
                        const double v_ang = Norm3(s.tcp_twist, 3);
                        if (!std::isfinite(v_lin) || !std::isfinite(v_ang)) {
                            Trip(stop_reason, StopReason::StateInvalid);
                            return;
                        }
                        if (v_lin > kMaxTcpLinearSpeed ||
                            v_ang > kMaxTcpAngularSpeed) {
                            Trip(stop_reason, StopReason::OverspeedTCP);
                            return;
                        }

                        double dp2 = 0.0;
                        for (size_t i = 0; i < 3; ++i) {
                            const double d = s.tcp_pose[i] - pose0[i];
                            dp2 += d * d;
                        }
                        if (!std::isfinite(dp2)) {
                            Trip(stop_reason, StopReason::StateInvalid);
                            return;
                        }
                        if (dp2 > kMaxTcpDisplacement * kMaxTcpDisplacement) {
                            Trip(stop_reason, StopReason::WorkspaceExceeded);
                            return;
                        }

                        if (s.q.size() != q0.size() ||
                            s.dtheta.size() != q0.size()) {
                            Trip(stop_reason, StopReason::StateInvalid);
                            return;
                        }
                        for (size_t i = 0; i < q0.size(); ++i) {
                            if (!std::isfinite(s.q[i]) ||
                                !std::isfinite(s.dtheta[i])) {
                                Trip(stop_reason, StopReason::StateInvalid);
                                return;
                            }
                            if (s.q[i] < q_min[i] + kJointLimitMargin ||
                                s.q[i] > q_max[i] - kJointLimitMargin) {
                                Trip(stop_reason, StopReason::JointLimitNearby);
                                return;
                            }
                            if (std::abs(s.dtheta[i]) >
                                kJointVelocityFraction * dq_max[i]) {
                                Trip(stop_reason, StopReason::OverspeedJoint);
                                return;
                            }
                        }

                        robot.StreamCartesianMotionForce(cmds);
                    } catch (...) {
                        Trip(stop_reason, StopReason::StreamFailed);
                    }
                },
                "RT Cartesian Floating", 1, scheduler.max_priority());

            std::cout << "Mode: "
                      << (opts.passive ? "6-axis passive zero-wrench"
                                       : "6-axis Cartesian reduced-stiffness")
                      << "\nStiffness scale: " << opts.scale
                      << "\nNull-space tracking weight: " << opts.nullspace
                      << "\nTCP local workspace radius: " << kMaxTcpDisplacement
                      << " m\nPress Ctrl+C to stop.\n";

            scheduler.Start();
            scheduler_started = true;

            // Ensure nominal RT streaming is active before releasing stiffness.
            const auto warmup_deadline = Clock::now() + std::chrono::seconds(2);
            while (!g_sigint && last_tick_ns.load() == 0 &&
                   stop_reason.load() == StopReason::None &&
                   Clock::now() < warmup_deadline) {
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }
            if (g_sigint || stop_reason.load() != StopReason::None ||
                last_tick_ns.load() == 0) {
                throw std::runtime_error("RT stream failed during warmup");
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            if (stop_reason.load() != StopReason::None ||
                NowNs() - last_tick_ns.load() > kHeartbeatTimeoutNs) {
                throw std::runtime_error("Unstable RT stream during warmup");
            }

            // Transition to the selected floating strategy while commands stream.
            // The human operator must support the robot during this transition.
            if (opts.passive) {
                robot.SetForceControlFrame(group, rdk::CoordType::WORLD);
                // Pure passive force control: all 6 axes, zero target wrench.
                robot.SetForceControlAxis(
                    group, {true, true, true, true, true, true});
                // WARNING: force-axis velocity protection is NOT effective
                // in passive force control. Software checks are not safety rated.
            } else {
                auto K = info.K_x_nom.at(group);
                for (auto& k : K) {
                    k *= opts.scale;
                }
                // Nominal damping ratio (0.7) retained.
                robot.SetCartesianImpedance(group, K);
            }

            const auto start = Clock::now();
            while (!g_sigint && stop_reason.load() == StopReason::None) {
                const auto last = last_tick_ns.load();
                if (last == 0) {
                    if (Clock::now() - start > std::chrono::seconds(2)) {
                        Trip(stop_reason, StopReason::HeartbeatLost);
                    }
                } else if (NowNs() - last > kHeartbeatTimeoutNs) {
                    Trip(stop_reason, StopReason::HeartbeatLost);
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }

            const auto reason = stop_reason.load();
            std::cerr << "Stopping: " << ReasonText(reason) << '\n';
            const bool stopped_ok = shutdown();
            return (stopped_ok && reason == StopReason::None) ? 0 : 1;

        } catch (...) {
            shutdown();
            throw;
        }
    } catch (const std::exception& e) {
        std::cerr << "Fatal error: " << e.what() << '\n';
        return 1;
    }
}
