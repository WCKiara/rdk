// ARM_1 Home + ARM_2 Hold, then paired RT commands: ARM_1 tilt/force + ARM_2 hold.
#include <flexiv/rdk/robot.hpp>
#include <flexiv/rdk/scheduler.hpp>
#include <Eigen/Geometry>
#include "stop_and_confirm.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <exception>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

namespace tilt_force {
namespace rdk = flexiv::rdk;
using Clock = std::chrono::steady_clock;
using Pose = std::array<double, 7>;
constexpr auto kArm1 = rdk::JointGroup::ARM_1;
constexpr auto kArm2 = rdk::JointGroup::ARM_2;
constexpr double kPi = 3.14159265358979323846;
constexpr std::array<bool, 6> kMotionOnly {};
constexpr std::array<bool, 6> kForceZ {false, false, true, false, false, false};
std::atomic<int> interrupted {0};
static_assert(std::atomic<int>::is_always_lock_free);
void Interrupt(int signum) { interrupted.store(signum); }
double Seconds(Clock::time_point start) { return std::chrono::duration<double>(Clock::now() - start).count(); }

struct Options {
    std::string serial;
    int axis = 0;
    double angle = 15, tilt_duration = 5, force = 5, force_duration = 6;
    double home_timeout = 60, startup_timeout = 10, finish_timeout = 3;
    double stop_timeout = 3;
    double force_velocity = 0.005;
    bool preview = false, tilt_only = false;
};

void Help()
{
    std::cout << "Usage: arm1_tcp_tilt_force ROBOT_SN [options]\n"
        "  --x DEG | --y DEG | --z DEG  LOCAL TCP rotation axis (default --x 15).\n"
        "  --tilt-duration SEC        Minimum smooth tilt duration (default 5).\n"
        "  --force N                  Exert force toward TCP +Z (default 5; signed).\n"
        "                             SDK sensed Fz target = -N.\n"
        "  --force-duration SEC       Stream constant force for this time (default 6).\n"
        "  --force-velocity M/S       SDK force-axis velocity setting (default 0.005).\n"
        "  --home-timeout SEC         ARM_1 Home deadline (default 60).\n"
        "  --startup-timeout SEC      Scheduler readiness deadline (default 10).\n"
        "  --finish-timeout SEC       Extra task completion budget (default 3).\n"
        "  --stop-timeout SEC         Confirmation budget after SDK Stop (default 3; >0.2).\n"
        "  --arm ARM_1                Only ARM_1 performs Home/tilt/force.\n"
        "  --preview                  Current-pose reference; no Home or writes.\n"
        "  --tilt-only                Home and tilt, then stop before force.\n"
        "Home sends {ARM_1: Home, ARM_2: Hold} together.\n"
        "Every 1000 Hz RT command contains ARM_1's target and ARM_2's fixed pose hold.\n"
        "Completion joins the callback, calls Stop, then confirms IDLE/all-stopped for 0.2 s.\n"
        "No added joint-drift, TCP-box, measured-wrench or force-tracking aborts.\n";
}

Options Parse(int argc, char** argv)
{
    if (argc < 2 || argv[1][0] == '-') throw std::invalid_argument("Robot serial is required");
    Options opt;
    opt.serial = argv[1];
    bool axis_set = false;
    for (int i = 2; i < argc; ++i) {
        const std::string arg = argv[i];
        const auto value = [&]() -> std::string {
            if (++i >= argc) throw std::invalid_argument("Missing value for " + arg);
            return argv[i];
        };
        const auto number = [&]() {
            const auto input = value();
            size_t used = 0;
            double result;
            try { result = std::stod(input, &used); }
            catch (const std::exception&) { throw std::invalid_argument("Expected a number for " + arg); }
            if (used != input.size() || !std::isfinite(result))
                throw std::invalid_argument("Expected a finite number for " + arg);
            return result;
        };
        if (arg == "--x" || arg == "--y" || arg == "--z") {
            if (axis_set) throw std::invalid_argument("Choose one of --x, --y, --z");
            axis_set = true;
            opt.axis = arg == "--x" ? 0 : arg == "--y" ? 1 : 2;
            opt.angle = number();
        } else if (arg == "--force") opt.force = number();
        else if (arg == "--tilt-duration") opt.tilt_duration = number();
        else if (arg == "--force-duration") opt.force_duration = number();
        else if (arg == "--force-velocity") opt.force_velocity = number();
        else if (arg == "--home-timeout") opt.home_timeout = number();
        else if (arg == "--startup-timeout") opt.startup_timeout = number();
        else if (arg == "--finish-timeout") opt.finish_timeout = number();
        else if (arg == "--stop-timeout") opt.stop_timeout = number();
        else if (arg == "--preview") opt.preview = true;
        else if (arg == "--tilt-only") opt.tilt_only = true;
        else if (arg == "--arm") {
            if (value() != "ARM_1") throw std::invalid_argument("Only --arm ARM_1 is supported");
        } else throw std::invalid_argument("Unknown option: " + arg);
    }
    for (double duration : {opt.tilt_duration, opt.force_duration, opt.home_timeout,
             opt.startup_timeout, opt.finish_timeout}) {
        if (duration <= 0) throw std::invalid_argument("Durations must be positive");
    }
    if (std::abs(opt.angle) > 180)
        throw std::invalid_argument("Use an angle within [-180,180] for the shortest rotation");
    if (opt.force_velocity < 0.005 || opt.force_velocity > 2)
        throw std::invalid_argument("SDK force velocity range is [0.005,2] m/s");
    if (opt.stop_timeout <= 0.2)
        throw std::invalid_argument("Stop confirmation timeout must exceed 0.2 seconds");
    return opt;
}

Eigen::Quaterniond Rotation(const Pose& pose)
{
    for (double value : pose) if (!std::isfinite(value)) throw std::runtime_error("Invalid TCP pose");
    const Eigen::Quaterniond q(pose[3], pose[4], pose[5], pose[6]);
    if (!std::isfinite(q.norm()) || q.norm() < 1e-8) throw std::runtime_error("Invalid TCP quaternion");
    return q.normalized();
}

struct TiltPlan {
    Pose start, target;
    double duration;
    TiltPlan(const Pose& initial, const Options& opt) : start(initial), target(initial), duration(opt.tilt_duration)
    {
        const auto q0 = Rotation(initial);
        const Eigen::Quaterniond q1 = (q0 * Eigen::AngleAxisd(opt.angle * kPi / 180,
            Eigen::Vector3d::Unit(opt.axis))).normalized();
        start[3] = q0.w(); start[4] = q0.x(); start[5] = q0.y(); start[6] = q0.z();
        target[3] = q1.w(); target[4] = q1.x(); target[5] = q1.y(); target[6] = q1.z();
        const double angle = q0.angularDistance(q1);
        duration = std::max({duration, 1.875 * angle / 0.15,
            std::sqrt((10 / std::sqrt(3.0)) * angle / 0.3)});
    }
    Pose At(double elapsed) const
    {
        const double u = std::clamp(elapsed / duration, 0.0, 1.0);
        const double s = u * u * u * (10 + u * (-15 + 6 * u));
        const auto q = Rotation(start).slerp(s, Rotation(target)).normalized();
        auto result = start;
        result[3] = q.w(); result[4] = q.x(); result[5] = q.y(); result[6] = q.z();
        return result;
    }
    bool Reached(const Pose& actual) const
    {
        const Eigen::Vector3d delta(actual[0] - target[0], actual[1] - target[1], actual[2] - target[2]);
        return delta.norm() <= 0.001 && Rotation(actual).angularDistance(Rotation(target)) <= kPi / 180;
    }
};

bool PrimitiveFlag(const rdk::PrimitiveStates& state, const char* key)
{
    const auto found = state.names_and_values.find(key);
    return found != state.names_and_values.end() && std::holds_alternative<int>(found->second)
        && std::get<int>(found->second) != 0;
}

void HomeArm1(rdk::Robot& robot, const Options& opt)
{
    rdk::PrimitiveArgs home("Home", {}), hold("Hold", {});
    for (auto* command : {&home, &hold}) {
        command->external_axis_control = false;
        command->sync_motion_mode = rdk::SyncMotionMode::DISABLE;
    }
    robot.SwitchMode(rdk::Mode::NRT_PRIMITIVE_EXECUTION);
    const auto begin = Clock::now();
    robot.ExecutePrimitive({{kArm1, home}, {kArm2, hold}}, true);
    std::cout << "HOME_STARTED ARM_1=Home ARM_2=Hold\n";
    while (true) {
        if (interrupted.load()) throw std::runtime_error("Interrupted during Home");
        if (robot.fault()) throw std::runtime_error("Robot fault during Home");
        if (Seconds(begin) > opt.home_timeout) throw std::runtime_error("ARM_1 Home timeout");
        const auto primitives = robot.primitive_states();
        const auto first = primitives.find(kArm1);
        if (first != primitives.end() && first->second.pt_name == "Home") {
            if (PrimitiveFlag(first->second, "reachedTarget")) break;
            if (PrimitiveFlag(first->second, "terminated"))
                throw std::runtime_error("ARM_1 Home terminated before arrival");
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    robot.SwitchMode(rdk::Mode::IDLE);
    std::cout << "HOME_COMPLETE ARM_1 reachedTarget=1\n";
}

enum class Stage { Gated, Tilt, AwaitForce, Force, Done, Failed };
struct Task {
    rdk::Robot& robot;
    const Options& opt;
    const TiltPlan& plan;
    std::map<rdk::JointGroup, rdk::RtCartesianCmd> commands;
    std::atomic<Stage> stage {Stage::Gated};
    std::atomic<bool> scheduler_ready {false};
    std::exception_ptr failure;
    uint64_t tilt_tick = 0;
    bool force_started = false;
    Clock::time_point force_begin;
    Task(rdk::Robot& robot, const Options& opt, const TiltPlan& plan, const Pose& hold)
    : robot(robot), opt(opt), plan(plan), commands {{kArm1, rdk::RtCartesianCmd(plan.start)},
          {kArm2, rdk::RtCartesianCmd(hold)}} {}

    void Tick() noexcept
    {
        scheduler_ready.store(true);
        const auto phase = stage.load();
        if (phase == Stage::Gated || phase == Stage::Done || phase == Stage::Failed) return;
        try {
            if (interrupted.load()) throw std::runtime_error("Interrupted");
            if (robot.fault()) throw std::runtime_error("Robot fault during RT control");
            auto& command = commands.at(kArm1);
            if (phase == Stage::Tilt) {
                command.pose_d = plan.At(tilt_tick * 0.001);
                if (tilt_tick * 0.001 >= plan.duration && plan.Reached(robot.states().at(kArm1).tcp_pose))
                    stage.store(opt.tilt_only ? Stage::Done : Stage::AwaitForce);
                ++tilt_tick;
            } else if (phase == Stage::Force) {
                if (!force_started) { force_started = true; force_begin = Clock::now(); }
                if (Seconds(force_begin) >= opt.force_duration) {
                    stage.store(Stage::Done);
                    return; // Never send another force packet after the deadline.
                }
                command.wrench_d[2] = -opt.force;
            }
            // The same ARM_2 pose/zero wrench accompanies EVERY ARM_1 RT target.
            robot.StreamCartesianMotionForce(commands);
        } catch (...) {
            failure = std::current_exception(); stage.store(Stage::Failed);
        }
    }
};

void PrintPose(const char* label, const Pose& pose)
{
    std::cout << label << std::setprecision(17);
    for (double value : pose) std::cout << ' ' << value;
    std::cout << '\n';
}

int Run(const Options& opt)
{
    std::unique_ptr<rdk::Scheduler> scheduler;
    if (!opt.preview) scheduler = std::make_unique<rdk::Scheduler>();
    rdk::Robot robot(opt.serial);
    const auto info = robot.info();
    if (!info.single_arm_groups.contains(kArm1) || !info.single_arm_groups.contains(kArm2))
        throw std::runtime_error("This task requires ARM_1 and ARM_2");
    if (!robot.connected() || robot.fault()) throw std::runtime_error("Robot disconnected or faulted");
    const auto initial = robot.states();
    auto plan = TiltPlan(initial.at(kArm1).tcp_pose, opt);
    const auto print_plan = [&] {
        PrintPose("ARM_1_START xyz_qw_qx_qy_qz", plan.start);
        PrintPose("ARM_1_TILT_TARGET xyz_qw_qx_qy_qz", plan.target);
        PrintPose("ARM_2_HOLD xyz_qw_qx_qy_qz", initial.at(kArm2).tcp_pose);
    };
    if (opt.preview) {
        std::cout << "PREVIEW_ONLY HOME_NOT_EXECUTED: current-pose reference; live tilt starts after Home\n";
        print_plan(); return 0;
    }
    if (!robot.operational() || robot.mode() != rdk::Mode::IDLE || !robot.all_stopped())
        throw std::runtime_error("Start READY, IDLE and stopped");
    Task task(robot, opt, plan, initial.at(kArm2).tcp_pose);
    bool started = false, control_started = false;
    std::exception_ptr error;
    try {
        scheduler->AddTask([&task] { task.Tick(); }, "arm1_task_arm2_hold", 1, scheduler->max_priority());
        scheduler->Start(); started = true;
        const auto startup = Clock::now();
        while (!task.scheduler_ready.load()) {
            if (interrupted.load() || Seconds(startup) > opt.startup_timeout)
                throw std::runtime_error("Scheduler readiness interrupted/timeout");
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        std::cout << "RT_SCHEDULER_READY\n";
        if (interrupted.load()) throw std::runtime_error("Interrupted before Home");
        control_started = true;
        HomeArm1(robot, opt);
        const auto after_home = robot.states();
        plan = TiltPlan(after_home.at(kArm1).tcp_pose, opt);
        task.commands.at(kArm1).pose_d = plan.start;
        print_plan();
        robot.SetPassiveForceControl(kArm1, false);
        robot.SwitchMode(rdk::Mode::RT_CARTESIAN_MOTION_FORCE);
        for (auto arm : {kArm1, kArm2}) {
            robot.SetForceControlAxis(arm, kMotionOnly);
            robot.SetCartesianImpedance(arm, info.K_x_nom.at(arm));
            robot.SetNullSpacePosture(arm, arm == kArm1 ? after_home.at(arm).q : initial.at(arm).q);
        }
        robot.SetNullSpaceObjectives(kArm2, 0, 0, 1.0);
        robot.SetForceControlFrame(kArm1, rdk::CoordType::TCP);
        if (interrupted.load()) throw std::runtime_error("Interrupted before tilt");
        auto phase_begin = Clock::now();
        task.stage.store(Stage::Tilt);
        while (true) {
            const auto phase = task.stage.load();
            if (phase == Stage::Done || phase == Stage::Failed) break;
            if (interrupted.load()) throw std::runtime_error("Interrupted");
            if (phase == Stage::AwaitForce) {
                robot.SetForceControlAxis(kArm1, kForceZ,
                    {opt.force_velocity, opt.force_velocity, opt.force_velocity});
                phase_begin = Clock::now();
                auto expected = Stage::AwaitForce;
                task.stage.compare_exchange_strong(expected, Stage::Force);
                std::cout << "TCP_Z_FORCE_ENABLED sensed_Fz_target_N=" << -opt.force << std::endl;
            } else if (Seconds(phase_begin) > (phase == Stage::Force ? opt.force_duration : plan.duration)
                    + opt.finish_timeout) {
                throw std::runtime_error(phase == Stage::Force ? "Force task timeout" : "Tilt arrival timeout");
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    } catch (...) { error = std::current_exception(); }

    task.stage.store(Stage::Gated);
    if (started) {
        try { scheduler->Stop(); } catch (...) { if (!error) error = std::current_exception(); }
    }
    scheduler.reset(); // Joins any remaining callback before task/robot destruction.
    if (task.failure) error = task.failure;
    if (control_started) {
        try {
            example::StopAndConfirmIdle(robot, opt.stop_timeout);
            std::cout << "STOP_CONFIRMED mode=IDLE all_stopped=1 stable_seconds=0.2\n";
        } catch (const std::exception& stop_error) {
            std::cerr << "STOP_UNCONFIRMED: " << stop_error.what() << '\n';
            if (!error) error = std::current_exception();
        }
    }
    if (error) {
        try {
            for (const auto& event : robot.event_log())
                std::cerr << "ROBOT_EVENT id=" << event.id << " description=" << event.description
                          << " causes=" << event.probable_causes << " actions=" << event.recommended_actions << '\n';
        } catch (...) {}
        std::rethrow_exception(error);
    }
    std::cout << (opt.tilt_only ? "TILT_COMPLETE" : "FORCE_TASK_COMPLETE") << " IDLE_POSITION_HOLD\n";
    return 0;
}
} // namespace tilt_force

int main(int argc, char** argv)
{
    for (int i = 1; i < argc; ++i) {
        if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
            tilt_force::Help(); return 0;
        }
    }
    try {
        const auto options = tilt_force::Parse(argc, argv);
        tilt_force::interrupted.store(0);
        std::signal(SIGINT, tilt_force::Interrupt);
        std::signal(SIGTERM, tilt_force::Interrupt);
        std::signal(SIGHUP, tilt_force::Interrupt);
        return tilt_force::Run(options);
    } catch (const std::exception& error) {
        std::cerr << "ARM1_TILT_FORCE_FAILED: " << error.what() << '\n';
        const int signum = tilt_force::interrupted.load();
        return signum ? 128 + signum : 1;
    }
}
