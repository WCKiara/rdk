// ARM_1 Home/FloatingJoint with a paired ARM_2 Hold in primitive execution mode.
#include <flexiv/rdk/robot.hpp>
#include "stop_and_confirm.hpp"

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <exception>
#include <iostream>
#include <map>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace arm1_floating {
namespace rdk = flexiv::rdk;
using Clock = std::chrono::steady_clock;
constexpr auto kArm1 = rdk::JointGroup::ARM_1;
constexpr auto kArm2 = rdk::JointGroup::ARM_2;
std::atomic<int> interrupted {0};
static_assert(std::atomic<int>::is_always_lock_free);
void Interrupt(int signal) { interrupted.store(signal); }
double Seconds(Clock::time_point begin)
{ return std::chrono::duration<double>(Clock::now() - begin).count(); }

struct Options {
    std::string serial;
    // Dimensionless primitive levels, NOT the RT example's torque damping gains.
    std::vector<double> damping_level = std::vector<double>(7, 0.0);
    double max_velocity = 0.2, duration = 0;
    double home_timeout = 60, startup_timeout = 10, stop_timeout = 3;
    bool preview = false, auto_servo_on = false, skip_home = false;
};

void Help()
{
    std::cout << "Usage: arm1_joint_floating_arm2_hold ROBOT_SN [options]\n"
        "  --damping-level D1 D2 D3 D4 D5 D6 D7  Levels [0,100]; default all 0.\n"
        "  --max-velocity M/S     Primitive maxVel [0.2,3], default 0.2.\n"
        "  --duration SEC         Floating time; 0 waits for Ctrl+C (default 0).\n"
        "  --skip-home            Float from the current position without Home.\n"
        "  --home-timeout SEC     ARM_1 Home budget (default 60).\n"
        "  --startup-timeout SEC  Servo readiness budget (default 10).\n"
        "  --stop-timeout SEC     Confirmation after SDK Stop (default 3; >0.2).\n"
        "  --auto-servo-on        Permit ServoOn only from NOT_SERVO_ON.\n"
        "  --arm ARM_1           Only ARM_1 can Home or float.\n"
        "  --preview             Read current state and print commands; no writes.\n"
        "Start in AUTO (Remote), IDLE and stopped.\n"
        "Commands: {ARM_1: Home, ARM_2: Hold}, then {ARM_1: FloatingJoint, ARM_2: Hold}.\n"
        "External axis control and synchronized motion are disabled.\n";
}

Options Parse(int argc, char** argv)
{
    if (argc < 2 || argv[1][0] == '-') throw std::invalid_argument("Robot serial is required");
    Options opt; opt.serial = argv[1];
    for (int i = 2; i < argc; ++i) {
        const std::string arg = argv[i];
        const auto value = [&]() -> std::string {
            if (++i >= argc) throw std::invalid_argument("Missing value for " + arg);
            return argv[i];
        };
        const auto number = [&]() {
            const auto input = value(); size_t used = 0;
            const double result = std::stod(input, &used);
            if (used != input.size() || !std::isfinite(result))
                throw std::invalid_argument("Expected a finite number for " + arg);
            return result;
        };
        if (arg == "--damping-level") for (double& level : opt.damping_level) level = number();
        else if (arg == "--max-velocity") opt.max_velocity = number();
        else if (arg == "--duration") opt.duration = number();
        else if (arg == "--home-timeout") opt.home_timeout = number();
        else if (arg == "--startup-timeout") opt.startup_timeout = number();
        else if (arg == "--stop-timeout") opt.stop_timeout = number();
        else if (arg == "--preview") opt.preview = true;
        else if (arg == "--skip-home") opt.skip_home = true;
        else if (arg == "--auto-servo-on") opt.auto_servo_on = true;
        else if (arg == "--arm") {
            if (value() != "ARM_1") throw std::invalid_argument("Only ARM_1 may Home or float");
        } else throw std::invalid_argument("Unknown argument: " + arg);
    }
    for (double level : opt.damping_level)
        if (level < 0 || level > 100) throw std::invalid_argument("dampingLevel range is [0,100]");
    if (opt.max_velocity < 0.2 || opt.max_velocity > 3 || opt.duration < 0
        || opt.home_timeout <= 0 || opt.startup_timeout <= 0 || opt.stop_timeout <= 0.2)
        throw std::invalid_argument("Invalid velocity, duration or timeout");
    return opt;
}

std::map<rdk::JointGroup, rdk::PrimitiveArgs> Pair(rdk::PrimitiveArgs arm1)
{
    std::map<rdk::JointGroup, rdk::PrimitiveArgs> result {
        {kArm1, std::move(arm1)}, {kArm2, rdk::PrimitiveArgs("Hold", {})}};
    for (auto& [_, command] : result) {
        command.external_axis_control = false;
        command.sync_motion_mode = rdk::SyncMotionMode::DISABLE;
    }
    return result;
}

void Check(rdk::Robot& robot)
{
    if (interrupted.load()) throw std::runtime_error("Interrupted");
    if (!robot.connected() || robot.fault()) throw std::runtime_error("Robot disconnected or faulted");
    if (!robot.operational()) throw std::runtime_error("Robot is not READY in AUTO (Remote)");
    if (robot.mode() != rdk::Mode::NRT_PRIMITIVE_EXECUTION)
        throw std::runtime_error("Primitive execution mode was lost");
}

bool Flag(const rdk::PrimitiveStates& state, const char* name)
{
    const auto found = state.names_and_values.find(name);
    return found != state.names_and_values.end() && std::holds_alternative<int>(found->second)
        && std::get<int>(found->second) != 0;
}

// Allow the asynchronous state report to catch up after ExecutePrimitive returns.
bool Matched(const std::map<rdk::JointGroup, rdk::PrimitiveStates>& states,
    const char* arm1_name, Clock::time_point begin, bool already_matched)
{
    const auto arm2 = states.find(kArm2);
    if (arm2 != states.end() && arm2->second.pt_name == "Hold" && Flag(arm2->second, "terminated"))
        throw std::runtime_error("ARM_2 Hold terminated");
    const auto arm1 = states.find(kArm1);
    if (arm1 != states.end() && arm1->second.pt_name == arm1_name
        && arm2 != states.end() && arm2->second.pt_name == "Hold") return true;
    if (already_matched || Seconds(begin) > 0.5)
        throw std::runtime_error("Expected ARM_1 primitive and ARM_2 Hold are not both active");
    return false;
}

int Run(const Options& opt)
{
    rdk::Robot robot(opt.serial);
    const auto info = robot.info();
    if (!info.single_arm_groups.contains(kArm1) || !info.single_arm_groups.contains(kArm2))
        throw std::runtime_error("ARM_1 and ARM_2 are both required");
    const auto initial = robot.states();
    for (const auto arm : {kArm1, kArm2}) {
        const auto found = initial.find(arm);
        if (found == initial.end() || found->second.q.size() != 7)
            throw std::runtime_error("Missing seven-joint state");
        for (double q : found->second.q)
            if (!std::isfinite(q)) throw std::runtime_error("Non-finite joint state");
    }
    std::cout << "PLAN ARM_1=" << (opt.skip_home ? "FloatingJoint" : "Home->FloatingJoint")
              << " ARM_2=Hold external_axis_control=0 dampingLevel=";
    for (double level : opt.damping_level) std::cout << level << ' ';
    std::cout << "maxVel=" << opt.max_velocity << " m/s\n" << std::flush;
    if (opt.preview) { std::cout << "PREVIEW_ONLY no control requests\n"; return 0; }
    if (!robot.connected() || robot.fault() || robot.mode() != rdk::Mode::IDLE || !robot.all_stopped())
        throw std::runtime_error("Start connected, fault-free, IDLE and stopped");

    bool control_started = false;
    std::exception_ptr error;
    try {
        if (interrupted.load()) throw std::runtime_error("Interrupted before control");
        if (!robot.operational()) {
            if (!opt.auto_servo_on || robot.operational_status() != rdk::OperationalStatus::NOT_SERVO_ON)
                throw std::runtime_error("Select AUTO (Remote) and READY before starting");
            control_started = true; robot.ServoOn();
            const auto startup = Clock::now();
            while (!robot.operational()) {
                if (interrupted.load() || robot.fault() || !robot.connected()
                    || Seconds(startup) > opt.startup_timeout)
                    throw std::runtime_error("Servo readiness failed/interrupted/timeout");
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }
        control_started = true;
        robot.SwitchMode(rdk::Mode::NRT_PRIMITIVE_EXECUTION);
        Check(robot);
        if (!opt.skip_home) {
            robot.ExecutePrimitive(Pair(rdk::PrimitiveArgs("Home", {})), true);
            std::cout << "HOME_STARTED ARM_1=Home ARM_2=Hold\n" << std::flush;
            const auto begin = Clock::now(); bool matched = false;
            while (true) {
                Check(robot);
                if (Seconds(begin) > opt.home_timeout) throw std::runtime_error("ARM_1 Home timeout");
                const auto states = robot.primitive_states();
                if (Matched(states, "Home", begin, matched)) {
                    matched = true; const auto& home = states.at(kArm1);
                    if (Flag(home, "reachedTarget") && robot.all_stopped()) break;
                    if (Flag(home, "terminated") && !Flag(home, "reachedTarget"))
                        throw std::runtime_error("ARM_1 Home terminated before arrival");
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
            std::cout << "HOME_COMPLETE ARM_1 reachedTarget=1 ARM_2=Hold\n" << std::flush;
        }
        Check(robot);
        robot.ExecutePrimitive(Pair(rdk::PrimitiveArgs("FloatingJoint", {
            {"floatingJoint", std::vector<double>(7, 1.0)},
            {"dampingLevel", opt.damping_level}, {"maxVel", opt.max_velocity}})), true);
        const auto begin = Clock::now(); bool matched = false;
        while (true) {
            Check(robot);
            const auto states = robot.primitive_states();
            if (Matched(states, "FloatingJoint", begin, matched)) {
                if (Flag(states.at(kArm1), "terminated")) {
                    if (!matched) throw std::runtime_error("ARM_1 FloatingJoint terminated before activation");
                    break;
                }
                if (!matched) std::cout << "FLOATING_ACTIVE ARM_1=FloatingJoint ARM_2=Hold\n" << std::flush;
                matched = true;
            }
            if (opt.duration > 0 && Seconds(begin) >= opt.duration) {
                if (!matched) throw std::runtime_error("ARM_1 FloatingJoint did not activate within duration");
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    } catch (...) { error = std::current_exception(); }
    if (control_started) {
        try {
            example::StopAndConfirmIdle(robot, opt.stop_timeout);
            std::cout << "STOP_CONFIRMED mode=IDLE all_stopped=1 stable_seconds=0.2\n" << std::flush;
        } catch (const std::exception& stop_error) {
            std::cerr << "STOP_UNCONFIRMED: " << stop_error.what() << '\n';
            if (!error) error = std::current_exception();
        }
    }
    if (error) std::rethrow_exception(error);
    std::cout << "FLOATING_FINISHED IDLE_POSITION_HOLD\n";
    return 0;
}
} // namespace arm1_floating

int main(int argc, char** argv)
{
    for (int i = 1; i < argc; ++i) if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
        arm1_floating::Help(); return 0;
    }
    try {
        const auto opt = arm1_floating::Parse(argc, argv);
        arm1_floating::interrupted = 0;
        std::signal(SIGINT, arm1_floating::Interrupt);
        std::signal(SIGTERM, arm1_floating::Interrupt);
        std::signal(SIGHUP, arm1_floating::Interrupt);
        return arm1_floating::Run(opt);
    } catch (const std::exception& error) {
        std::cerr << "ARM1_FLOATING_FAILED: " << error.what() << '\n';
        const int signal = arm1_floating::interrupted.load();
        return signal ? 128 + signal : 1;
    }
}
