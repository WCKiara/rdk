// Compile with tests/fake_rdk first on the include path. The actual motion
// implementation is exercised against in-memory fake classes; no SDK is linked.
#include <unistd.h>
inline uid_t offline_geteuid() { return 0; }
#define geteuid offline_geteuid
#define main MotionExampleMain
#include "../third_party/flexiv_rdk/example/intermediate6_realtime_cartesian_motion_force_control.cpp"
#undef main
#undef geteuid

#include <algorithm>
#include <functional>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace {
void Require(bool condition, const std::string& message)
{
    if (!condition) throw std::runtime_error(message);
}

void Reset()
{
    rdk::fake = rdk::FakeRobotData {};
    rdk::ResetFakeScheduler();
    g_interrupted = 0;
}

int Invoke(const std::vector<std::string>& extra = {})
{
    std::vector<std::string> arguments {"offline-example", "Offline-LL-654321", "100",
        "--arm", "ARM_1", "--delta", "0.0001", "0", "-0.0001", "--tilt", "0", "0.01", "0",
        "--duration", "0.006", "--max-linear-vel", "1", "--max-angular-vel", "1",
        "--max-linear-acc", "100", "--max-angular-acc", "100",
        "--finish-timeout", "0.04", "--startup-timeout", "0.25", "--home-timeout", "0.04"};
    arguments.insert(arguments.end(), extra.begin(), extra.end());
    std::vector<char*> argv;
    for (auto& argument : arguments) argv.push_back(argument.data());
    return MotionExampleMain(static_cast<int>(argv.size()), argv.data());
}

struct InvocationResult {
    int code;
    std::string errors;
};

InvocationResult InvokeWithErrors(const std::vector<std::string>& extra = {})
{
    std::ostringstream errors;
    auto* previous = std::cerr.rdbuf(errors.rdbuf());
    const int result = Invoke(extra);
    std::cerr.rdbuf(previous);
    return {result, errors.str()};
}

void OnlyArm1Commands(bool servo_permitted = false)
{
    for (const auto& command : rdk::fake.stream_commands) {
        Require(command.size() == 1 && command.contains(kArm), "stream addressed another group");
        for (double force : command.at(kArm).wrench_d)
            Require(force == 0, "pose stream commanded nonzero force");
    }
    for (const auto& command : rdk::fake.primitives) {
        Require(command.size() == 1 && command.contains(kArm), "Home addressed another group");
        Require(command.at(kArm).name == "Home", "unexpected primitive");
        Require(!command.at(kArm).external_axis_control, "Home enabled external axes");
        Require(command.at(kArm).sync_motion_mode == rdk::SyncMotionMode::DISABLE,
                "Home enabled synchronized group motion");
    }
    for (auto group : rdk::fake.axis_groups) Require(group == kArm, "force axis config addressed ARM_2");
    for (auto group : rdk::fake.wrench_groups) Require(group == kArm, "wrench config addressed ARM_2");
    if (!servo_permitted) {
        Require(std::find(rdk::fake.events.begin(), rdk::fake.events.end(), "servo") == rdk::fake.events.end(),
                "program issued robot-wide ServoOn without opt-in");
    }
    Require(std::count(rdk::fake.events.begin(), rdk::fake.events.end(), "servo") <= 1,
            "program retried ServoOn");
    if (rdk::fake.streams) {
        for (bool enabled : rdk::fake.force_axes) Require(!enabled, "explicit force control was enabled");
    }
}

void TestSuccessAndArm1OnlyHome()
{
    Reset();
    Require(Invoke({"--return-home"}) == 0, "successful trajectory failed");
    Require(rdk::fake.streams > 1, "real-time callbacks did not stream a trajectory");
    Require(rdk::fake.homes == 1, "expected one final Home");
    Require(rdk::fake.stops == 2, "success must stop after motion and Home");
    const auto& events = rdk::fake.events;
    const auto first_stream = std::find(events.begin(), events.end(), "stream");
    const auto first_stop = std::find(events.begin(), events.end(), "stop");
    const auto home = std::find(events.begin(), events.end(), "home");
    Require(first_stream < first_stop && first_stop < home, "Home began before successful stream + stop");
    Require(events.back() == "stop", "Home did not finish with Stop");
    Require(rdk::fake.first_control_state_reads >= 100
            && rdk::fake.first_control_arm2_time_ns >= 101000000,
            "control began before the 100 ms fresh ARM_2 position-history window");
    OnlyArm1Commands();
}

void TestSuccessWithoutHome()
{
    Reset();
    Require(Invoke() == 0, "successful stream failed");
    Require(rdk::fake.homes == 0 && rdk::fake.stops == 1, "Home was not optional");
    OnlyArm1Commands();
}

Pose MeasuredHomePose()
{
    Pose home{0.4011, 0.1996, 0.1213, 0.001538137, -0.000788480, 0.999998495, 0.000152865};
    const auto rotation = single_arm_rt::Rotation(home);
    home[3] = rotation.w(); home[4] = rotation.x();
    home[5] = rotation.y(); home[6] = rotation.z();
    return home;
}

void TestHomeTrajectoryPreservesMeasuredStart(bool sign_flip_during_configuration)
{
    Reset();
    const auto home = MeasuredHomePose();
    rdk::fake.states.at(kArm).tcp_pose = home;
    rdk::fake.actions.at(kArm).tcp_pose_d = home;
    if (sign_flip_during_configuration) {
        rdk::fake.after_switch_mode = [](rdk::Mode mode) {
            if (mode == rdk::Mode::RT_CARTESIAN_MOTION_FORCE) {
                auto& measured = rdk::fake.states.at(kArm).tcp_pose;
                for (size_t i = 3; i < measured.size(); ++i) measured[i] *= -1;
            }
        };
    }
    bool target_reached_before_home = false;
    rdk::fake.after_stop = [&] {
        if (rdk::fake.homes == 0 && !rdk::fake.stream_commands.empty()) {
            const auto& last = rdk::fake.stream_commands.back().at(kArm).pose_d;
            const double angle = single_arm_rt::Rotation(home).angularDistance(single_arm_rt::Rotation(last));
            target_reached_before_home = std::abs(angle - 15 * single_arm_rt::kPi / 180) < 1e-10
                && std::abs(last[0] - home[0] - 0.01) < 1e-12
                && std::abs(last[2] - home[2] + 0.01) < 1e-12;
        }
    };
    std::ostringstream output;
    auto* previous = std::cout.rdbuf(output.rdbuf());
    const int result = Invoke({"--delta", "0.01", "0", "-0.01", "--tilt", "0", "15", "0",
        "--finish-timeout", "0.5", "--return-home"});
    std::cout.rdbuf(previous);
    Require(result == 0, "measured Home quaternion trajectory failed");
    Require(rdk::fake.stream_commands.size() >= 5, "Home trajectory sent fewer than five samples");
    const auto& first = rdk::fake.stream_commands.front().at(kArm).pose_d;
    for (size_t i = 0; i < home.size(); ++i)
        Require(std::abs(first[i] - home[i]) < 1e-12,
                "first Home target changed measured start or quaternion sign during configuration");
    Require(std::abs(first[3]) < 0.01 && std::abs(first[5]) > 0.99,
            "first Home packet used identity orientation");
    for (size_t i = 1; i <= 4; ++i) {
        const auto& packet = rdk::fake.stream_commands[i].at(kArm).pose_d;
        Require(single_arm_rt::Rotation(home).angularDistance(single_arm_rt::Rotation(packet)) < 1e-5,
                "Home packet rotated abruptly in the first four milliseconds");
    }
    Require(target_reached_before_home && rdk::fake.homes == 1,
            "Home command began before completing the relative 15-degree trajectory");
    Require(output.str().find("START_CAPTURED") != std::string::npos,
            "trajectory output omitted captured measured start");
    OnlyArm1Commands();
}

void TestConfigurationPoseJumpIsRejected(bool orientation_jump)
{
    Reset();
    rdk::fake.states.at(kArm).tcp_pose = MeasuredHomePose();
    rdk::fake.after_switch_mode = [orientation_jump](rdk::Mode mode) {
        if (mode != rdk::Mode::RT_CARTESIAN_MOTION_FORCE) return;
        auto& measured = rdk::fake.states.at(kArm).tcp_pose;
        if (!orientation_jump) {
            measured[0] += 0.002;
        } else {
            const Eigen::Quaterniond rotated = single_arm_rt::Rotation(measured)
                * Eigen::AngleAxisd(2 * single_arm_rt::kPi / 180, Eigen::Vector3d::UnitZ());
            measured[3] = rotated.w(); measured[4] = rotated.x();
            measured[5] = rotated.y(); measured[6] = rotated.z();
        }
    };
    const auto result = InvokeWithErrors({"--return-home"});
    Require(result.code != 0, "RT configuration pose jump was silently replanned");
    Require(std::find(rdk::fake.events.begin(), rdk::fake.events.end(), "mode:rt")
            != rdk::fake.events.end(), "configuration jump fixture did not reach RT mode");
    Require(rdk::fake.streams == 0 && rdk::fake.stops >= 1 && rdk::fake.homes == 0,
            "configuration pose jump sent targets or Home before aborting");
    OnlyArm1Commands();
}

void TestControllerOrientationFaultIncludesEventsAndBothArmActions()
{
    Reset();
    rdk::fake.states.at(kArm).tcp_pose = MeasuredHomePose();
    rdk::fake.actions.at(kOtherArm).tcp_pose_d = Pose{0.433333, 0.222222, 0.166666, 1, 0, 0, 0};
    rdk::RobotEvent event;
    event.id = 9015;
    event.level = rdk::RobotEvent::ERROR;
    event.description = "orientation error exceeds 90 degrees (offline controller event)";
    event.consequences = "offline consequence: controller enters MINOR_FAULT";
    event.probable_causes = "offline cause: desired and measured TCP orientation differ";
    event.recommended_actions = "offline recommendation: inspect both arms before retry";
    event.timestamp = std::chrono::system_clock::time_point(std::chrono::seconds(123456789));
    rdk::fake.event_log = {event};
    rdk::fake.after_stream = [] {
        if (rdk::fake.streams == 4) {
            rdk::fake.fault = true;
            rdk::fake.operational = false;
            rdk::fake.status = rdk::OperationalStatus::MINOR_FAULT;
        }
    };
    const auto result = InvokeWithErrors({"--return-home"});
    Require(result.code != 0 && rdk::fake.streams == 4,
            "controller fault after four returned commands was not detected");
    Require(rdk::fake.stops >= 1 && rdk::fake.homes == 0,
            "controller orientation fault did not Stop without Home");
    Require(rdk::fake.constructions == 1 && rdk::fake.action_reads > 0 && rdk::fake.event_log_reads > 0,
            "fault diagnostics did not read actions/event log from the existing robot");
    Require(rdk::fake.event_log_reads_before_stop == 0,
            "controller event log was read before attempting Robot::Stop");
    for (const auto* detail : {"orientation error exceeds 90 degrees", "9015", "offline consequence",
            "offline cause", "offline recommendation", "ROBOT_POSE_SNAPSHOT: ARM_2", "0.433333",
            "controller_desired", "controller_orientation_error_deg=", "measured [x y z qw qx qy qz]:"})
        Require(result.errors.find(detail) != std::string::npos,
                "controller fault diagnostic omitted: " + std::string(detail));
    OnlyArm1Commands();
}

void TestFailureDoesNotHome(const std::function<void()>& inject, const std::string& name)
{
    Reset();
    rdk::fake.after_stream = inject;
    Require(Invoke({"--return-home"}) != 0, name + " was not detected");
    Require(rdk::fake.streams >= 1, name + " fixture never reached streaming");
    Require(rdk::fake.homes == 0, name + " incorrectly started Home");
    Require(rdk::fake.stops >= 1, name + " did not Stop");
    OnlyArm1Commands();
}

void TestArm2RejectedBeforeConstruction()
{
    Reset();
    std::vector<std::string> arguments {"offline-example", "Offline-LL-654321", "--arm", "ARM_2",
                                      "--delta", "0", "0", "0"};
    std::vector<char*> argv;
    for (auto& argument : arguments) argv.push_back(argument.data());
    Require(MotionExampleMain(static_cast<int>(argv.size()), argv.data()) != 0, "ARM_2 option accepted");
    Require(rdk::fake.constructions == 0, "ARM_2 rejection connected a Robot first");
}

void TestPreviewHasNoControlCalls()
{
    Reset();
    Require(Invoke({"--preview", "--return-home"}) == 0, "preview failed");
    Require(rdk::fake.events == std::vector<std::string> {"construct"}, "preview sent a control command");
}

void TestInterruptionBetweenStopAndHome()
{
    Reset();
    rdk::fake.after_stop = [] { g_interrupted = 1; };
    Require(Invoke({"--return-home"}) == 130, "post-stream interruption was not detected");
    Require(rdk::fake.homes == 0, "post-stream interruption started Home");
    OnlyArm1Commands();
}

void TestStableArm2QIgnoresRawDqNoise()
{
    Reset();
    rdk::fake.states.at(rdk::JointGroup::ARM_2).dq = {0.02, -0.04, 0.09, -0.075, 0.025, -0.06, 0.03};
    Require(Invoke({"--return-home"}) == 0, "stable ARM_2 q was rejected because of noisy raw dq");
    Require(rdk::fake.streams > 1 && rdk::fake.homes == 1 && rdk::fake.stops == 2,
            "raw dq noise regression did not complete ARM_1 motion/Home lifecycle");
    Require(rdk::fake.first_control_state_reads >= 100
            && rdk::fake.first_control_arm2_time_ns >= 101000000,
            "noisy dq fixture bypassed fresh ARM_2 position-history window");
    OnlyArm1Commands();
}

void TestStartupArm2DriftBlocksControl()
{
    Reset();
    rdk::fake.after_states = [] {
        if (rdk::fake.state_reads == 3) rdk::fake.states.at(rdk::JointGroup::ARM_2).q[3] = 0.002;
    };
    std::ostringstream errors;
    auto* previous_err = std::cerr.rdbuf(errors.rdbuf());
    const int result = Invoke({"--return-home"});
    std::cerr.rdbuf(previous_err);
    Require(result != 0, "startup ARM_2 q drift was accepted");
    Require(rdk::fake.events == std::vector<std::string> {"construct"}, "startup ARM_2 q drift sent control commands");
    for (const auto* detail : {"startup", "joint=4", "delta_q=0.002",
             "ARM_2 is monitored only; no motion target is sent to it"}) {
        Require(errors.str().find(detail) != std::string::npos,
                "startup ARM_2 diagnostic omitted: " + std::string(detail));
    }
}

void TestNonfiniteStartupDqBlocksControl()
{
    Reset();
    rdk::fake.states.at(rdk::JointGroup::ARM_2).dq[3] = std::numeric_limits<double>::quiet_NaN();
    std::ostringstream errors;
    auto* previous_err = std::cerr.rdbuf(errors.rdbuf());
    const int result = Invoke({"--return-home"});
    std::cerr.rdbuf(previous_err);
    Require(result != 0, "nonfinite ARM_2 raw dq was accepted");
    Require(rdk::fake.events == std::vector<std::string> {"construct"},
            "nonfinite ARM_2 raw dq sent control commands");
    for (const auto* detail : {"Invalid ARM_2 raw dq [startup snapshot]", "joint=4", "raw_dq=nan"}) {
        Require(errors.str().find(detail) != std::string::npos,
                "startup invalid raw dq diagnostic omitted: " + std::string(detail));
    }
}

void TestNotOperationalBlocksControl()
{
    Reset();
    rdk::fake.operational = false;
    rdk::fake.status = rdk::OperationalStatus::NOT_SERVO_ON;
    Require(Invoke({"--return-home"}) != 0, "non-operational robot was accepted");
    Require(rdk::fake.events == std::vector<std::string> {"construct"},
            "non-operational robot received control/servo commands");
}

void TestFrozenTimestampBlocksControl(bool arm1)
{
    Reset();
    rdk::fake.advance_arm1_stamp = !arm1;
    rdk::fake.advance_arm2_stamp = arm1;
    Require(Invoke({"--return-home"}) != 0, "frozen startup timestamp was accepted");
    Require(rdk::fake.events == std::vector<std::string> {"construct"},
            "frozen startup timestamp sent control commands");
}

void RequireNoRealtimeControl(const std::string& context)
{
    Require(std::find(rdk::fake.events.begin(), rdk::fake.events.end(), "mode")
            == rdk::fake.events.end(), context + " switched robot mode");
    Require(rdk::fake.streams == 0 && rdk::fake.homes == 0,
            context + " sent trajectory or Home targets");
    Require(rdk::fake.axis_groups.empty() && rdk::fake.wrench_groups.empty(),
            context + " configured robot force control");
}

void RequireCallbackDiagnostic(const InvocationResult& result, const std::string& context)
{
    Require(result.errors.find("callbacks=") != std::string::npos
            && result.errors.find("command_attempts=") != std::string::npos
            && result.errors.find("commands=") != std::string::npos,
            context + " omitted callback/command attempt/success counts");
}

void TestSchedulerConstructorPrecedesRobot()
{
    Reset();
    rdk::fake_scheduler_constructor_delay_ms = 60;
    bool constructed_before_robot = false;
    rdk::fake_scheduler_after_constructor = [&] {
        constructed_before_robot = rdk::fake.constructions == 0;
    };
    Require(Invoke({"--return-home"}) == 0, "delayed scheduler constructor broke motion");
    Require(constructed_before_robot, "scheduler initialization happened after Robot construction");
    Require(rdk::fake.streams > 1 && rdk::fake.homes == 1,
            "delayed constructor did not complete trajectory/Home");
    OnlyArm1Commands();
}

void TestSchedulerConstructorFailureDoesNotConnect()
{
    Reset();
    rdk::fake_scheduler_throw_in_constructor = true;
    const auto result = InvokeWithErrors({"--auto-servo-on", "--return-home"});
    Require(result.code != 0 && result.errors.find("injected scheduler constructor failure")
            != std::string::npos, "scheduler constructor failure diagnostic was lost");
    Require(rdk::fake.constructions == 0 && rdk::fake.events.empty(),
            "scheduler constructor failure connected or controlled the robot");
}

void TestDelayedSchedulerStartDoesNotConsumeMotionTime()
{
    Reset();
    // Longer than both the 20 ms watchdog and duration + finish-timeout.
    rdk::fake_scheduler_start_delay_ms = 70;
    bool started_idle = false;
    rdk::fake_scheduler_before_start = [&] {
        started_idle = rdk::fake.mode == rdk::Mode::IDLE
            && rdk::fake.axis_groups.empty() && rdk::fake.wrench_groups.empty();
    };
    Require(Invoke({"--return-home"}) == 0, "blocking scheduler Start consumed trajectory time");
    Require(started_idle, "scheduler Start happened after switching RT mode or force configuration");
    Require(rdk::fake.streams > 1 && rdk::fake.homes == 1,
            "delayed scheduler Start did not complete trajectory/Home");
    OnlyArm1Commands();
}

void TestDelayedFirstCallbackUsesStartupTimeout()
{
    Reset();
    rdk::fake_scheduler_first_callback_delay_ms = 60;
    bool first_callback_idle = false;
    rdk::fake_scheduler_before_first_callback = [&] {
        first_callback_idle = rdk::fake.mode == rdk::Mode::IDLE
            && rdk::fake.streams == 0 && rdk::fake.axis_groups.empty()
            && rdk::fake.wrench_groups.empty();
    };
    Require(Invoke({"--return-home"}) == 0, "first callback used runtime heartbeat timeout during startup");
    Require(first_callback_idle, "robot control started before first scheduler callback");
    Require(rdk::fake.streams > 1 && rdk::fake.homes == 1,
            "delayed first callback did not complete trajectory/Home");
    OnlyArm1Commands();
}

void TestMissingCallbacksLeavesReadyRobotIdle()
{
    Reset();
    rdk::fake_scheduler_skip_callbacks = true;
    const auto result = InvokeWithErrors({"--return-home", "--startup-timeout", "0.15"});
    Require(result.code != 0, "scheduler without callbacks hung or succeeded");
    RequireNoRealtimeControl("missing startup callbacks");
    Require(rdk::fake.events == std::vector<std::string> {"construct"},
            "missing startup callbacks sent control/Stop to already READY robot");
    Require(result.errors.find("callback") != std::string::npos
            && result.errors.find("startup") != std::string::npos,
            "missing callback startup timeout omitted callback/startup diagnostic");
    RequireCallbackDiagnostic(result, "missing startup callbacks");
}

void TestMissingCallbacksAfterServoStopsWithoutMode()
{
    Reset();
    rdk::fake.operational = false;
    rdk::fake.status = rdk::OperationalStatus::NOT_SERVO_ON;
    rdk::fake_scheduler_skip_callbacks = true;
    Require(Invoke({"--auto-servo-on", "--return-home", "--startup-timeout", "0.15"}) != 0,
            "missing startup callbacks after ServoOn succeeded");
    RequireNoRealtimeControl("missing callbacks after ServoOn");
    Require(rdk::fake.events == std::vector<std::string> {"construct", "servo", "stop"},
            "missing startup callbacks after ServoOn did not clean up with Stop");
    OnlyArm1Commands(true);
}

void TestSchedulerStartFailureBeforeMode(bool auto_servo)
{
    Reset();
    if (auto_servo) {
        rdk::fake.operational = false;
        rdk::fake.status = rdk::OperationalStatus::NOT_SERVO_ON;
    }
    rdk::fake_scheduler_throw_on_start = true;
    const auto result = InvokeWithErrors(auto_servo
        ? std::vector<std::string> {"--auto-servo-on", "--return-home"}
        : std::vector<std::string> {"--return-home"});
    Require(result.code != 0 && result.errors.find("injected scheduler Start failure")
            != std::string::npos, "scheduler Start failure diagnostic was lost");
    RequireNoRealtimeControl("scheduler Start failure");
    const auto expected = auto_servo
        ? std::vector<std::string> {"construct", "servo", "stop"}
        : std::vector<std::string> {"construct"};
    Require(rdk::fake.events == expected, "scheduler Start failure used incorrect Stop/ServoOn cleanup");
    OnlyArm1Commands(auto_servo);
}

void TestRuntimeCallbackPauseStillStops()
{
    Reset();
    rdk::fake.after_stream = [] { rdk::fake_scheduler_pause_callbacks_ms.store(60); };
    const auto result = InvokeWithErrors({"--return-home"});
    Require(result.code != 0, "runtime callback pause was accepted");
    Require(rdk::fake.streams >= 1 && rdk::fake.stops >= 1 && rdk::fake.homes == 0,
            "runtime callback pause did not Stop without Home");
    Require(result.errors.find("heartbeat") != std::string::npos,
            "runtime callback pause lost the heartbeat diagnostic");
    RequireCallbackDiagnostic(result, "runtime callback pause");
    OnlyArm1Commands();
}

void TestSpecificCallbackErrorSurvivesWatchdog()
{
    Reset();
    rdk::fake.after_stream = [] {
        // While the callback is blocked, the supervisory heartbeat watchdog fires.
        // Scheduler::Stop joins this callback; its specific exception must survive.
        std::this_thread::sleep_for(std::chrono::milliseconds(60));
        throw std::runtime_error("specific injected StreamCartesianMotionForce failure");
    };
    const auto result = InvokeWithErrors({"--return-home"});
    Require(result.code != 0, "blocked failing callback was accepted");
    Require(rdk::fake.streams >= 1 && rdk::fake.stops >= 1 && rdk::fake.homes == 0,
            "blocked failing callback did not Stop without Home");
    Require(result.errors.find("specific injected StreamCartesianMotionForce failure")
            != std::string::npos, "watchdog replaced the callback's specific exception");
    RequireCallbackDiagnostic(result, "specific callback failure");
    Require(result.errors.find("command_attempts=1") != std::string::npos
            && result.errors.find("commands=0") != std::string::npos,
            "failed SDK command was counted as successful or never attempted");
    OnlyArm1Commands();
}

void TestSchedulerStopFailure(const std::function<void()>& after_stream,
    const std::string& diagnostic, const std::string& context)
{
    Reset();
    rdk::fake_scheduler_throw_on_stop.store(true);
    rdk::fake.after_stream = after_stream;
    bool scheduler_cleaned_before_robot_stop = false;
    rdk::fake.after_stop = [&] {
        scheduler_cleaned_before_robot_stop = rdk::fake_scheduler_joined.load()
            && rdk::fake_scheduler_destroyed.load();
    };
    const auto result = InvokeWithErrors({"--return-home"});
    Require(result.code != 0, context + " was accepted");
    Require(!rdk::fake_scheduler_throw_on_stop.load(), context + " never exercised Scheduler::Stop");
    Require(rdk::fake.streams >= 1 && rdk::fake.stops >= 1 && rdk::fake.homes == 0,
            context + " did not Stop robot without Home");
    Require(scheduler_cleaned_before_robot_stop,
            context + " stopped robot before destroying/joining scheduler callbacks");
    Require(result.errors.find(diagnostic) != std::string::npos,
            context + " lost the specific failure diagnostic");
    RequireCallbackDiagnostic(result, context);
    OnlyArm1Commands();
}

void TestFailureToConfirmStopBlocksHome()
{
    Reset();
    rdk::fake.stop_confirms = false;
    Require(Invoke({"--return-home"}) != 0, "failed Stop confirmation was accepted");
    Require(rdk::fake.homes == 0, "Home began without IDLE/all-stopped confirmation");
}

void TestHomeFailureStopsAgain()
{
    Reset();
    rdk::fake.home_reached = false;
    Require(Invoke({"--return-home"}) != 0, "Home termination without arrival was accepted");
    Require(rdk::fake.homes == 1 && rdk::fake.stops == 2, "Home failure did not stop again");
    OnlyArm1Commands();
}

void NotServoOn()
{
    rdk::fake.operational = false;
    rdk::fake.status = rdk::OperationalStatus::NOT_SERVO_ON;
}

void TestAutoServoWaitsAndReplans()
{
    Reset();
    NotServoOn();
    int wait_samples = 0;
    rdk::fake.after_servo = [] {
        rdk::fake.operational = false;
        rdk::fake.status = rdk::OperationalStatus::RELEASING_BRAKE;
        rdk::fake.states.at(kArm).tcp_pose[0] = 0.402;
    };
    rdk::fake.after_states = [&] {
        if (rdk::fake.status == rdk::OperationalStatus::RELEASING_BRAKE && ++wait_samples == 3) {
            rdk::fake.operational = true;
            rdk::fake.status = rdk::OperationalStatus::READY;
        }
    };
    Require(Invoke({"--auto-servo-on", "--return-home"}) == 0, "automatic servo startup failed");
    Require(wait_samples == 3, "program did not wait for operational READY");
    const auto& events = rdk::fake.events;
    Require(std::count(events.begin(), events.end(), "servo") == 1, "expected one ServoOn");
    Require(std::find(events.begin(), events.end(), "servo") < std::find(events.begin(), events.end(), "mode"),
            "mode changed before servo enable");
    Require(std::abs(rdk::fake.stream_commands.front().at(kArm).pose_d[0] - 0.402) < 1e-10,
            "first target used the pre-enable start pose");
    Require(std::abs(rdk::fake.stream_commands.back().at(kArm).pose_d[0] - 0.4021) < 1e-10,
            "delta was not replanned from the post-enable pose");
    Require(rdk::fake.homes == 1, "successful servo/motion sequence did not return Home");
    OnlyArm1Commands(true);
}

void TestAutoServoAlreadyReady()
{
    Reset();
    Require(Invoke({"--auto-servo-on"}) == 0, "already READY startup failed");
    OnlyArm1Commands();
}

void TestAutoServoPreviewRemainsReadonly()
{
    Reset();
    NotServoOn();
    Require(Invoke({"--auto-servo-on", "--preview", "--return-home"}) == 0, "auto-servo preview failed");
    Require(rdk::fake.events == std::vector<std::string> {"construct"}, "preview issued controls");
}

void TestAutoServoFailure(const std::function<void()>& after_servo, const std::string& name,
    bool interrupted = false)
{
    Reset();
    NotServoOn();
    rdk::fake.after_servo = after_servo;
    const int result = Invoke({"--auto-servo-on", "--return-home", "--clear-minor-fault"});
    Require(result == (interrupted ? 130 : 1), name + " was not reported");
    Require(rdk::fake.events == std::vector<std::string> {"construct", "servo", "stop"},
            name + " did not stop before mode/trajectory/Home");
    Require(rdk::fake.streams == 0 && rdk::fake.homes == 0 && rdk::fake.fault_clears == 0,
            name + " streamed, homed or cleared a runtime fault");
    OnlyArm1Commands(true);
}

void TestAutoServoRejectsOtherStatuses()
{
    for (auto status : {rdk::OperationalStatus::ESTOP_NOT_RELEASED, rdk::OperationalStatus::BOOTING,
            rdk::OperationalStatus::IN_MANUAL_MODE, rdk::OperationalStatus::IN_AUTO_MODE,
            rdk::OperationalStatus::IN_REDUCED_STATE, rdk::OperationalStatus::IN_RECOVERY_STATE}) {
        Reset();
        rdk::fake.operational = false;
        rdk::fake.status = status;
        Require(Invoke({"--auto-servo-on"}) != 0, "non-enablable status was accepted");
        Require(rdk::fake.events == std::vector<std::string> {"construct"},
                "non-enablable status received controls");
    }
}

void TestAutoServoMinorFaultRecovery()
{
    Reset();
    NotServoOn();
    rdk::fake.fault = true;
    rdk::fake.status = rdk::OperationalStatus::MINOR_FAULT;
    Require(Invoke({"--auto-servo-on", "--clear-minor-fault"}) == 0,
            "explicit minor-fault recovery followed by ServoOn failed");
    Require(rdk::fake.fault_clears == 1, "minor fault was not cleared once");
    Require(std::find(rdk::fake.events.begin(), rdk::fake.events.end(), "clear-fault")
            < std::find(rdk::fake.events.begin(), rdk::fake.events.end(), "servo"),
            "ServoOn preceded minor-fault recovery");
    OnlyArm1Commands(true);
}

void TestAutoServoCriticalFaultRejected()
{
    Reset();
    NotServoOn();
    rdk::fake.fault = true;
    rdk::fake.status = rdk::OperationalStatus::CRITICAL_FAULT;
    Require(Invoke({"--auto-servo-on", "--clear-minor-fault"}) != 0, "critical fault was accepted");
    Require(rdk::fake.events == std::vector<std::string> {"construct"}, "critical fault received controls");
}

void TestInvalidTargetDoesNotEnable()
{
    Reset();
    NotServoOn();
    Require(Invoke({"--auto-servo-on", "--delta", "1", "0", "0"}) != 0,
            "invalid target was accepted");
    Require(rdk::fake.events == std::vector<std::string> {"construct"},
            "invalid target enabled the robot");
}
} // namespace

int main()
{
    const auto previous_int = std::signal(SIGINT, SIG_DFL);
    const auto previous_term = std::signal(SIGTERM, SIG_DFL);
    const auto previous_hup = std::signal(SIGHUP, SIG_DFL);
    std::ostringstream transcript;
    auto* previous_out = std::cout.rdbuf(transcript.rdbuf());
    auto* previous_err = std::cerr.rdbuf(transcript.rdbuf());
    try {
        TestSuccessAndArm1OnlyHome();
        TestSuccessWithoutHome();
        TestHomeTrajectoryPreservesMeasuredStart(false);
        TestHomeTrajectoryPreservesMeasuredStart(true);
        TestConfigurationPoseJumpIsRejected(false);
        TestConfigurationPoseJumpIsRejected(true);
        TestControllerOrientationFaultIncludesEventsAndBothArmActions();
        TestArm2RejectedBeforeConstruction();
        TestPreviewHasNoControlCalls();
        TestFailureDoesNotHome([] { rdk::fake.fault = true; }, "runtime fault");
        TestFailureDoesNotHome([] { g_interrupted = 1; }, "runtime interruption");
        TestFailureDoesNotHome([] { rdk::fake.states.at(rdk::JointGroup::ARM_2).q[0] = 0.01; }, "ARM_2 drift");
        TestFailureDoesNotHome([] {
            rdk::fake.states.at(rdk::JointGroup::ARM_2).dq[0] = std::numeric_limits<double>::quiet_NaN();
        }, "nonfinite ARM_2 raw dq");
        TestFailureDoesNotHome([] { rdk::fake.states.at(kArm).tcp_pose[2] = 0.19; }, "position bounds");
        TestFailureDoesNotHome([] { rdk::fake.states.at(kArm).tcp_wrench[0] = 11; }, "contact wrench");
        TestFailureDoesNotHome([] { rdk::fake.connected = false; }, "disconnect");
        TestFailureDoesNotHome([] { rdk::fake.operational = false; }, "lost operational state");
        TestInterruptionBetweenStopAndHome();
        TestStableArm2QIgnoresRawDqNoise();
        TestStartupArm2DriftBlocksControl();
        TestNonfiniteStartupDqBlocksControl();
        TestNotOperationalBlocksControl();
        TestFrozenTimestampBlocksControl(true);
        TestFrozenTimestampBlocksControl(false);
        TestSchedulerConstructorPrecedesRobot();
        TestSchedulerConstructorFailureDoesNotConnect();
        TestDelayedSchedulerStartDoesNotConsumeMotionTime();
        TestDelayedFirstCallbackUsesStartupTimeout();
        TestMissingCallbacksLeavesReadyRobotIdle();
        TestMissingCallbacksAfterServoStopsWithoutMode();
        TestSchedulerStartFailureBeforeMode(false);
        TestSchedulerStartFailureBeforeMode(true);
        TestRuntimeCallbackPauseStillStops();
        TestSpecificCallbackErrorSurvivesWatchdog();
        TestSchedulerStopFailure({}, "injected scheduler Stop failure", "successful motion / scheduler Stop failure");
        TestSchedulerStopFailure([] { rdk::fake_scheduler_pause_callbacks_ms.store(60); },
            "heartbeat timeout", "runtime watchdog / scheduler Stop failure");
        TestSchedulerStopFailure([] { throw std::runtime_error("specific callback / Stop failure"); },
            "specific callback / Stop failure", "callback exception / scheduler Stop failure");
        TestFailureToConfirmStopBlocksHome();
        TestHomeFailureStopsAgain();
        TestAutoServoWaitsAndReplans();
        TestAutoServoAlreadyReady();
        TestAutoServoPreviewRemainsReadonly();
        TestAutoServoFailure([] {
            rdk::fake.operational = false;
            rdk::fake.status = rdk::OperationalStatus::RELEASING_BRAKE;
        }, "servo timeout");
        TestAutoServoFailure([] { g_interrupted = 1; }, "servo interruption", true);
        TestAutoServoFailure([] {
            rdk::fake.fault = true;
            rdk::fake.status = rdk::OperationalStatus::MINOR_FAULT;
        }, "fault during servo startup");
        TestAutoServoFailure([] { rdk::fake.connected = false; }, "disconnect during servo startup");
        TestAutoServoFailure([] { rdk::fake.states.at(kOtherArm).q[0] = 0.002; }, "ARM_2 servo drift");
        TestAutoServoFailure([] { rdk::fake.states.at(kArm).tcp_pose[2] = 0.19; }, "ARM_1 servo boundary");
        TestAutoServoFailure([] { rdk::fake.states.at(kArm).tcp_pose[0] = 0.45495; }, "post-servo target outside bounds");
        TestAutoServoFailure([] { throw std::runtime_error("request failed after delivery"); }, "ServoOn exception");
        TestAutoServoFailure([] {
            rdk::fake.operational = false;
            rdk::fake.status = rdk::OperationalStatus::RELEASING_BRAKE;
            rdk::fake.advance_arm1_stamp = false;
            rdk::fake.advance_arm2_stamp = false;
        }, "stale states during servo startup");
        TestAutoServoRejectsOtherStatuses();
        TestAutoServoMinorFaultRecovery();
        TestAutoServoCriticalFaultRejected();
        TestInvalidTargetDoesNotEnable();
        std::signal(SIGINT, previous_int);
        std::signal(SIGTERM, previous_term);
        std::signal(SIGHUP, previous_hup);
        std::cout.rdbuf(previous_out);
        std::cerr.rdbuf(previous_err);
        std::cout << "Realtime lifecycle cases (scheduler startup, watchdog and automatic ServoOn) passed against fake SDK; no hardware access" << std::endl;
        return 0;
    } catch (const std::exception& error) {
        std::signal(SIGINT, previous_int);
        std::signal(SIGTERM, previous_term);
        std::signal(SIGHUP, previous_hup);
        std::cout.rdbuf(previous_out);
        std::cerr.rdbuf(previous_err);
        std::cerr << transcript.str();
        std::cerr << "LIFECYCLE TEST FAILED: " << error.what() << std::endl;
        return 1;
    }
}
