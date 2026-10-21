// Compile with tests/fake_polishing_rdk first on the include path and SDK data
// headers after it. No real Robot/Scheduler implementation or SDK library is used.
// Run each case in a fresh process (the example's callback counter is static).
#define main PolishingExampleMain
#include "../third_party/flexiv_rdk/example/intermediate6_realtime_cartesian_motion_force_control.cpp"
#undef main

#include <vector>
#include <limits>

namespace {
int Invoke(const std::vector<std::string>& options)
{
    std::vector<std::string> arguments {"offline-polishing-test", "Offline-LL-654321"};
    arguments.insert(arguments.end(), options.begin(), options.end());
    std::vector<char*> argv;
    for (auto& argument : arguments) argv.push_back(argument.data());
    return PolishingExampleMain(static_cast<int>(argv.size()), argv.data());
}
} // namespace

int main(int argc, char** argv)
{
    using namespace rdk::polishing_test;
    try {
        Require(argc == 2, "specify one regression case");
        const std::string test = argv[1];
        state.stop_requested = [] { return g_stop_sched.load(); };
        std::vector<std::string> options {"--arm", "ARM_1", "--polish"};
        const bool uses_movel = test.starts_with("movel_") || test == "tilt_only_movel";
        const bool tilt_only = test.starts_with("tilt_only_");
        if (uses_movel) options.insert(options.end(), {"--tilt-method", "movel"});
        if (tilt_only) options.push_back("--tilt-only");
        bool success = false, no_control = false, no_connection = false;
        if (test == "dual_arm" || test == "tcp" || test == "already_ready" || test == "default_arm"
            || test == "movel_full" || test == "movel_near_pi" || test == "movel_negative_quat"
            || tilt_only) {
            success = true;
            if (test == "tcp") options.push_back("--TCP");
            if (test == "already_ready") options.push_back("--auto-servo-on");
            if (test == "default_arm") options = {"--polish"};
            if (test == "movel_near_pi" || test == "movel_negative_quat") {
                state.measured.at(kControlledArm).tcp_pose
                    = {0.4011, 0.1996, 0.1213, 0.001538137, -0.000788480, 0.999998495, 0.000152865};
                if (test == "movel_negative_quat") {
                    auto& pose = state.measured.at(kControlledArm).tcp_pose;
                    for (size_t i = 3; i < pose.size(); ++i) pose[i] *= -1;
                }
            }
        } else if (test == "missing_arm1") {
            state.info.single_arm_groups.erase(rdk::JointGroup::ARM_1);
            // Even explicit enabling must not run when the requested arm is absent.
            state.ready = false;
            state.status = rdk::OperationalStatus::NOT_SERVO_ON;
            options.push_back("--auto-servo-on");
            no_control = true;
        } else if (test == "arm2" || test == "all" || test == "missing_arm_value"
                   || test == "unknown_option" || test == "bad_tilt_method" || test == "missing_tilt_method") {
            if (test == "arm2") options = {"--arm", "ARM_2"};
            if (test == "all") options = {"--arm", "ALL"};
            if (test == "missing_arm_value") options = {"--arm"};
            if (test == "unknown_option") options = {"--position", "0", "0", "0"};
            if (test == "bad_tilt_method") options = {"--tilt-method", "invalid"};
            if (test == "missing_tilt_method") options = {"--tilt-method"};
            no_connection = no_control = true;
        } else if (test == "not_ready" || test == "servo_opt_in" || test == "servo_failure") {
            state.ready = false;
            state.status = rdk::OperationalStatus::NOT_SERVO_ON;
            if (test == "not_ready") no_control = true;
            else options.push_back("--auto-servo-on");
            if (test == "servo_opt_in") success = true;
            if (test == "servo_failure") state.fail_servo = true;
        } else if (test == "fault" || test == "active_mode" || test == "not_stopped"
                   || test == "other_not_ready") {
            no_control = true;
            if (test == "fault") state.fault = true;
            if (test == "active_mode") state.mode = rdk::Mode::RT_CARTESIAN_MOTION_FORCE;
            if (test == "not_stopped") state.stopped = false;
            if (test == "other_not_ready") {
                state.ready = false;
                state.status = rdk::OperationalStatus::ESTOP_NOT_RELEASED;
                options.push_back("--auto-servo-on");
            }
        } else if (test == "home_failure") {
            state.fail_home = true;
        } else if (test == "nrt_rejected" || test == "nrt_rejected_stop_failure") {
            state.fail_nrt_plan = true;
            state.fail_stop = test == "nrt_rejected_stop_failure";
        } else if (test == "movel_rejected") {
            state.fail_movel_plan = true;
        } else if (test == "movel_terminated") {
            state.movel_terminated = true;
        } else if (test == "movel_bad_joints") {
            state.measured.at(kControlledArm).q.clear();
        } else if (test == "bad_quaternion") {
            state.measured.at(kControlledArm).tcp_pose = {0.4, 0.2, 0.2, 0, 0, 0, 0};
        } else if (test == "nonfinite_pose") {
            state.measured.at(kControlledArm).tcp_pose[0] = std::numeric_limits<double>::quiet_NaN();
        } else {
            throw std::runtime_error("unknown test case: " + test);
        }
        const auto arm1_before = state.measured.at(kControlledArm).tcp_pose;
        const auto reference_q = state.measured.at(kControlledArm).q;
        const auto arm2_before = state.measured.at(rdk::JointGroup::ARM_2).tcp_pose;
        Require((Invoke(options) == 0) == success, "unexpected exit status");
        Require(state.measured.at(rdk::JointGroup::ARM_2).tcp_pose == arm2_before,
            "fake ARM_2 pose changed");
        Require(!no_connection || state.constructions == 0, "invalid arm connected to a robot");
        Require(!no_control || state.controls == 0, "rejected startup issued a control request");
        Require(state.servos == ((test == "servo_opt_in" || test == "servo_failure") ? 1 : 0),
            "ServoOn ran without a required explicit opt-in, or was retried");
        if (success) {
            Require(state.movels == (uses_movel ? 1 : 0), "wrong tilt planner or automatic retry");
            if (uses_movel) {
                Require(state.movel_reads == 3, "MoveL completion used a missing or wrong primitive/group");
                const auto expected = TiltPose15DegWorldY(arm1_before);
                Require(std::abs(PoseQuaternion(arm1_before).angularDistance(PoseQuaternion(expected))
                        - 15 * kPi / 180) < 1e-12, "tilt angle changed near pi or with negated quaternion");
                for (size_t i = 0; i < 3; ++i)
                    Require(state.movel_target.position[i] == arm1_before[i], "MoveL changed tilt TCP position");
                const auto& rpy = state.movel_target.orientation;
                const Eigen::Quaterniond rebuilt
                    = Eigen::AngleAxisd(rpy[2] * kPi / 180, Eigen::Vector3d::UnitZ())
                    * Eigen::AngleAxisd(rpy[1] * kPi / 180, Eigen::Vector3d::UnitY())
                    * Eigen::AngleAxisd(rpy[0] * kPi / 180, Eigen::Vector3d::UnitX());
                Require(rebuilt.angularDistance(PoseQuaternion(expected)) < 1e-12,
                    "MoveL degree/Euler conversion changed target orientation");
                for (size_t i = 0; i < reference_q.size(); ++i)
                    Require(std::abs(state.movel_target.ref_q_m[i] - reference_q[i] * 180 / kPi) < 1e-12,
                        "MoveL joint reference used the wrong unit or pose");
            }
            if (tilt_only) {
                Require(state.homes == 1 && state.zeros == 0 && state.streams == 0
                        && state.sends == (uses_movel ? 0 : 1) && state.stops == 1
                        && state.wrench_settings == 0 && state.frame_settings == 0 && state.axis_settings == 0,
                    "tilt-only continued to zero/contact/RT or omitted Stop");
                std::cout << "PASS " << test << '\n';
                return 0;
            }
            Require(state.homes == 1 && state.zeros == 1 && state.sends == (uses_movel ? 1 : 2)
                    && state.streams > 6000,
                "full Home/tilt/zero/search/RT workflow did not complete");
            Require(state.wrench_settings == 1 && state.frame_settings == 1 && state.axis_settings == 1,
                "missing ARM_1 per-arm configuration");
            Require(state.primitive_reads == 4, "zeroing used another group's completion");
            Require(state.stops == 1 && state.scheduler_stops == 1, "successful workflow did not stop");
            Require(std::abs(state.last_target[1] - state.line_start[1] - 0.05) < 1e-12,
                "polishing line endpoint changed");
            Require(state.last_wrench[2] == (test == "tcp" ? -5.0 : 5.0), "force frame changed");
            const auto expected_frame = test == "tcp" ? rdk::CoordType::TCP : rdk::CoordType::WORLD;
            Require(state.frame == expected_frame, "wrong force-control frame");
        }
        if (test == "home_failure" || test == "servo_failure") {
            Require(state.stops == 1 && state.sends == 0 && state.streams == 0,
                "failed startup continued motion or omitted Stop");
        }
        if (test == "nrt_rejected" || test == "nrt_rejected_stop_failure" || test == "movel_rejected"
            || test == "movel_terminated" || test == "movel_bad_joints" || test == "bad_quaternion"
            || test == "nonfinite_pose") {
            Require(state.stops == 1 && state.zeros == 0 && state.streams == 0 && state.event_reads == 1,
                "tilt failure continued to later phases or lost fault diagnostics");
            Require(state.sends == (state.fail_nrt_plan ? 1 : 0), "failed tilt retried direct NRT commands");
            Require(state.movels == ((state.fail_movel_plan || state.movel_terminated) ? 1 : 0),
                "failed tilt automatically changed planner or retried MoveL");
        }
        std::cout << "PASS " << test << '\n';
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "FAIL: " << e.what() << '\n';
        return 1;
    }
}
