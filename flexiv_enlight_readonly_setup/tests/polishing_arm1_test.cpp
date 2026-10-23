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
        if (test == "tcp_axis_geometry") {
            const Pose identity {0.4, 0.2, 0.1, 1, 0, 0, 0};
            const auto identity_target = AdvanceAlongTcpPositiveZ(identity);
            Require(identity_target[0] == 0.4 && identity_target[1] == 0.2
                    && std::abs(identity_target[2] - 0.15) < 1e-12,
                "identity TCP +Z is not WORLD +Z");
            const double half = std::sqrt(0.5);
            const auto side_target = AdvanceAlongTcpPositiveZ({0.4, 0.2, 0.1, half, 0, half, 0});
            Require(std::abs(side_target[0] - 0.45) < 1e-12 && side_target[1] == 0.2
                    && std::abs(side_target[2] - 0.1) < 1e-12,
                "90-degree TCP +Z is not WORLD +X");
            const auto down_target = AdvanceAlongTcpPositiveZ({0.4, 0.2, 0.1, 0, 0, 1, 0});
            Require(down_target[0] == 0.4 && down_target[1] == 0.2
                    && std::abs(down_target[2] - 0.05) < 1e-12,
                "upside-down TCP +Z is not WORLD -Z");
            const double half_angle = 195.0 * kPi / 360.0;
            Pose tilted {0.4, 0.2, 0.1, std::cos(half_angle), 0, std::sin(half_angle), 0};
            const auto tilted_target = AdvanceAlongTcpPositiveZ(tilted);
            Require(std::abs(tilted_target[0] - (0.4 - 0.05 * std::sin(kPi / 12))) < 1e-12
                    && tilted_target[1] == 0.2
                    && std::abs(tilted_target[2] - (0.1 - 0.05 * std::cos(kPi / 12))) < 1e-12,
                "tilted TCP +Z did not follow the tool axis toward WORLD -X/-Z");
            for (size_t i = 3; i < 7; ++i) tilted[i] *= -3.0;
            const auto equivalent_target = AdvanceAlongTcpPositiveZ(tilted);
            for (size_t i = 0; i < 3; ++i) {
                Require(std::abs(equivalent_target[i] - tilted_target[i]) < 1e-12,
                    "equivalent quaternion changed the TCP translation");
            }
            Require(state.constructions == 0 && state.controls == 0,
                "pure geometry check connected or issued control");
            std::cout << "PASS " << test << '\n';
            return 0;
        }
        state.stop_requested = [] { return g_stop_sched.load(); };
        std::vector<std::string> options {"--arm", "ARM_1", "--polish"};
        const bool uses_movel = test.starts_with("movel_") || test == "tilt_only_movel"
            || test == "descent_movel_both" || test == "descent_movel_logged_pose";
        const bool uses_descent_movel = test.starts_with("descent_movel_");
        const bool tilt_only = test.starts_with("tilt_only_");
        if (uses_movel) options.insert(options.end(), {"--tilt-method", "movel"});
        if (uses_descent_movel) options.insert(options.end(), {"--descent-method", "movel"});
        if (tilt_only) options.push_back("--tilt-only");
        bool success = false, no_control = false, no_connection = false;
        if (test == "dual_arm" || test == "tcp" || test == "already_ready" || test == "default_arm"
            || test == "movel_full" || test == "movel_near_pi" || test == "movel_negative_quat"
            || test == "world_tilt" || test == "hold_press" || test == "early_contact"
            || test == "descent_no_contact" || test == "descent_lateral_force"
            || test == "descent_negative_force" || test == "descent_tcp_force_only"
            || test == "descent_still_moving" || test == "descent_wrong_x"
            || test == "descent_wrong_rotation" || test == "descent_missing_stop"
            || test == "descent_movel_full" || test == "descent_movel_both"
            || test == "descent_movel_logged_pose" || test == "descent_movel_early_contact"
            || test == "descent_movel_reference" || test == "explicit_descent_nrt"
            || test == "capture_measured_tcp"
            || tilt_only) {
            success = true;
            if (test == "tcp") options.push_back("--TCP");
            if (test == "capture_measured_tcp") state.alter_zero_orientation = true;
            if (test == "already_ready") options.push_back("--auto-servo-on");
            if (test == "default_arm") options = {"--polish"};
            if (test == "hold_press") options = {"--arm", "ARM_1"};
            if (test == "early_contact" || test == "descent_movel_early_contact") state.contact_before_endpoint = true;
            if (test == "descent_movel_reference") state.update_zero_joints = true;
            if (test == "explicit_descent_nrt") options.insert(options.end(), {"--descent-method", "nrt"});
            if (test == "descent_movel_logged_pose") {
                state.measured.at(kControlledArm).tcp_pose = {0.40120524168014526,
                    0.19961023330688477, 0.12149646133184433, 0.0016360542504116893,
                    -0.00079422391718253493, 0.99999833106994629, -8.264313692052383e-06};
            }
            if (test == "descent_still_moving") state.descent_still_moving = true;
            if (test == "descent_wrong_x") state.descent_x_offset = true;
            if (test == "descent_wrong_rotation") state.descent_rotation_error = true;
            if (test == "descent_missing_stop") state.missing_descent_stop = true;
            if (test == "descent_no_contact" || test == "descent_lateral_force"
                || test == "descent_negative_force" || test == "descent_tcp_force_only") {
                state.descent_wrench = {};
                if (test == "descent_lateral_force") state.descent_wrench[0] = 5.0;
                if (test == "descent_negative_force") state.descent_wrench[2] = -4.0;
                if (test == "descent_tcp_force_only") state.descent_local_wrench[2] = 4.0;
            }
            if (test == "world_tilt") {
                const Eigen::Quaterniond orientation
                    = Eigen::AngleAxisd(0.7, Eigen::Vector3d::UnitZ())
                    * Eigen::AngleAxisd(-0.4, Eigen::Vector3d::UnitX());
                state.measured.at(kControlledArm).tcp_pose
                    = {0.4, 0.2, 0.2, orientation.w(), orientation.x(), orientation.y(), orientation.z()};
            }
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
                   || test == "unknown_option" || test == "bad_tilt_method" || test == "missing_tilt_method"
                   || test == "bad_descent_method" || test == "missing_descent_method") {
            if (test == "arm2") options = {"--arm", "ARM_2"};
            if (test == "all") options = {"--arm", "ALL"};
            if (test == "missing_arm_value") options = {"--arm"};
            if (test == "unknown_option") options = {"--position", "0", "0", "0"};
            if (test == "bad_tilt_method") options = {"--tilt-method", "invalid"};
            if (test == "missing_tilt_method") options = {"--tilt-method"};
            if (test == "bad_descent_method") options = {"--descent-method", "invalid"};
            if (test == "missing_descent_method") options = {"--descent-method"};
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
        } else if (test == "nonfinite_descent_force" || test == "nonfinite_descent_pose"
                   || test == "descent_rejected" || test == "descent_event_delay"
                   || test == "descent_movel_rejected" || test == "descent_movel_terminated"
                   || test == "descent_movel_bad_joints" || test == "descent_movel_force_limit"
                   || test == "descent_movel_moment_limit" || test == "nrt_force_limit"
                   || test == "descent_movel_nonfinite") {
            if (test == "nonfinite_descent_force") state.descent_wrench[2] = std::numeric_limits<double>::infinity();
            if (test == "nonfinite_descent_pose") state.nonfinite_descent_pose = true;
            if (test == "descent_rejected" || test == "descent_movel_rejected" || test == "descent_event_delay") state.fail_descent_plan = true;
            if (test == "descent_event_delay") state.event_delay_reads = 2;
            if (test == "descent_movel_terminated") state.descent_movel_terminated = true;
            if (test == "descent_movel_bad_joints") state.bad_descent_joints = true;
            if (test == "descent_movel_force_limit") state.descent_wrench[0] = 10.1;
            if (test == "descent_movel_moment_limit") state.descent_wrench[5] = 3.1;
            if (test == "nrt_force_limit") state.descent_wrench[2] = 10.1;
            if (test == "descent_movel_nonfinite") state.descent_wrench[0] = std::numeric_limits<double>::quiet_NaN();
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
            Require(state.descent_movels == (uses_descent_movel ? 1 : 0),
                "descent planner changed without an explicit option, or retried");
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
            Require(state.homes == 1 && state.zeros == 1
                    && state.sends == (uses_movel ? 0 : 1) + (uses_descent_movel ? 0 : 1)
                    && state.streams > 6000,
                "full Home/tilt/zero/descent/RT workflow did not complete");
            Require(state.wrench_settings == (uses_descent_movel ? 1 : 2)
                    && state.frame_settings == 1 && state.axis_settings == (uses_descent_movel ? 1 : 2),
                "missing ARM_1 per-arm configuration");
            Require(state.primitive_reads == 4, "zeroing used another group's completion");
            Require(state.stops == 1 && state.scheduler_stops == 1, "successful workflow did not stop");
            const double expected_line = test == "hold_press" ? 0.0 : 0.05;
            Require(std::abs(state.last_target[1] - state.line_start[1] - expected_line) < 1e-12,
                "polishing line endpoint changed");
            const Eigen::Vector3d world_displacement(state.descent_target[0] - state.descent_start[0],
                state.descent_target[1] - state.descent_start[1], state.descent_target[2] - state.descent_start[2]);
            const Eigen::Vector3d local_displacement
                = PoseQuaternion(state.descent_start).conjugate() * world_displacement;
            Require(std::abs(local_displacement.x()) < 1e-12 && std::abs(local_displacement.y()) < 1e-12
                    && std::abs(local_displacement.z() - 0.050) < 1e-12,
                "position move is not exactly 50 mm along the captured TCP +Z");
            Require(PoseQuaternion(state.descent_start).angularDistance(PoseQuaternion(state.descent_target)) < 1e-12,
                "descent changed the tilted orientation");
            const Eigen::Quaterniond expected_world_delta(std::cos(kPi / 24), 0, std::sin(kPi / 24), 0);
            const auto actual_delta = PoseQuaternion(state.after_tilt_pose) * PoseQuaternion(arm1_before).conjugate();
            Require(expected_world_delta.angularDistance(actual_delta) < 1e-12,
                "tilt is not +15 degrees about the fixed WORLD Y axis");
            Require(PoseQuaternion(state.last_target).angularDistance(PoseQuaternion(state.descent_target)) < 1e-12,
                "RT pressing changed the tilted orientation");
            Require(state.last_target[0] == state.line_start[0]
                    && state.last_target[2] == state.line_start[2],
                "RT pose did not retain the post-descent X/Z setpoint");
            Require(state.last_wrench == std::array<double, rdk::kCartDoF>{0, 0, -5, 0, 0, 0}
                    && state.frame == rdk::CoordType::TCP,
                "press is not TCP +Z 5 N (sensed-force target TCP Fz=-5 N)");
            const Eigen::Vector3d exerted_force_world = PoseQuaternion(state.descent_target)
                * Eigen::Vector3d(-state.last_wrench[0], -state.last_wrench[1], -state.last_wrench[2]);
            Require((exerted_force_world / 5.0 - world_displacement / 0.050).norm() < 1e-12,
                "pressing force is not aligned with the TCP advance direction");
            const Eigen::Vector3d completed_displacement(state.line_start[0] - state.descent_start[0],
                state.line_start[1] - state.descent_start[1], state.line_start[2] - state.descent_start[2]);
            Require((completed_displacement - world_displacement).norm() < 1e-12,
                "RT pressing started before the full 50 mm TCP displacement");
            if (test == "capture_measured_tcp") {
                Require(world_displacement.y() < -0.0002,
                    "position move used the nominal tilt target instead of the measured post-zero TCP axis");
                Require(PoseQuaternion(state.last_target).angularDistance(PoseQuaternion(state.descent_start)) < 1e-12,
                    "RT restored an earlier quaternion instead of retaining the captured TCP orientation");
            }
            if (test == "descent_movel_logged_pose") {
                const Eigen::Vector3d expected_world_delta(-0.0127828519, -0.00000069649067, -0.0483383771);
                Require((world_displacement - expected_world_delta).norm() < 1e-7,
                    "logged robot orientation did not move TCP +Z toward WORLD -X/-Z");
            }
            if (test == "early_contact" || test == "descent_still_moving"
                || test == "descent_wrong_x" || test == "descent_wrong_rotation"
                || test == "descent_missing_stop" || test == "descent_movel_early_contact") {
                Require(state.rt_entry_reads == 3,
                    "descent ended early on force, before position/orientation arrival or ARM_1 stop");
            }
            if (uses_descent_movel) {
                Require(state.descent_movel_reads == 3,
                    "descent MoveL completion used a missing or wrong primitive/group");
                for (size_t i = 0; i < state.descent_start_q.size(); ++i) {
                    Require(std::abs(state.descent_movel_target.ref_q_m[i]
                            - state.descent_start_q[i] * 180 / kPi) < 1e-12,
                        "descent MoveL used stale joints or incorrect joint units");
                }
            }
        }
        if (test == "nonfinite_descent_force" || test == "nonfinite_descent_pose" || test == "descent_rejected"
            || test == "descent_event_delay" || test == "descent_movel_rejected"
            || test == "descent_movel_terminated" || test == "descent_movel_bad_joints"
            || test == "descent_movel_force_limit" || test == "descent_movel_moment_limit"
            || test == "nrt_force_limit" || test == "descent_movel_nonfinite") {
            Require(state.stops == 1 && state.zeros == 1 && state.sends == (uses_descent_movel ? 1 : 2)
                    && state.streams == 0 && state.frame_settings == 0
                    && state.axis_settings == (uses_descent_movel ? 0 : 1)
                    && state.event_reads == state.event_delay_reads + 1,
                "failed position move started RT pressing or omitted Stop/diagnostics");
            Require(state.descent_movels == ((uses_descent_movel && test != "descent_movel_bad_joints") ? 1 : 0),
                "rejected descent retried or automatically changed planner");
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
