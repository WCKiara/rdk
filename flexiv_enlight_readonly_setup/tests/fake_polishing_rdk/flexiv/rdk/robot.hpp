#pragma once

// In-memory replacement: uses SDK data types, never links or loads the robot SDK.
#include <flexiv/rdk/data.hpp>
#include <flexiv/rdk/mode.hpp>
#include <Eigen/Geometry>
#include <functional>
#include <limits>
#include <stdexcept>

namespace flexiv::rdk {
namespace polishing_test {
struct Data {
    RobotInfo info;
    std::map<JointGroup, RobotStates> measured {{JointGroup::ARM_1, {}}, {JointGroup::ARM_2, {}}};
    bool ready = true, fault = false, stopped = true, fail_home = false, fail_servo = false;
    bool fail_nrt_plan = false, fail_movel_plan = false, fail_stop = false;
    bool movel_terminated = false;
    bool contact_before_endpoint = false, descent_active = false, descent_still_moving = false;
    bool fail_descent_plan = false, nonfinite_descent_pose = false;
    bool descent_movel_terminated = false, bad_descent_joints = false, update_zero_joints = false;
    bool alter_zero_orientation = false;
    bool descent_x_offset = false, descent_rotation_error = false, missing_descent_stop = false;
    Mode mode = Mode::IDLE;
    OperationalStatus status = OperationalStatus::READY;
    int constructions = 0, controls = 0, servos = 0, homes = 0, stops = 0;
    int sends = 0, streams = 0, zeros = 0, primitive_reads = 0;
    int movels = 0, movel_reads = 0, event_reads = 0;
    int descent_movels = 0, descent_movel_reads = 0, event_delay_reads = 0;
    int descent_reads = 0, rt_entry_reads = 0;
    std::string active_primitive;
    Coord movel_target;
    Coord descent_movel_target;
    std::vector<double> descent_start_q;
    int frame_settings = 0, axis_settings = 0, wrench_settings = 0, scheduler_stops = 0;
    CoordType frame = CoordType::WORLD;
    std::array<double, kPoseSize> line_start {}, last_target {};
    std::array<double, kPoseSize> descent_start {}, descent_target {};
    std::array<double, kPoseSize> after_tilt_pose {};
    std::array<double, kCartDoF> descent_wrench {0, 0, 4, 0, 0, 0};
    std::array<double, kCartDoF> descent_local_wrench {};
    std::array<double, kCartDoF> last_wrench {};
    std::function<bool()> stop_requested;
    Data()
    {
        info.single_arm_groups = {{JointGroup::ARM_1, "ARM_1"}, {JointGroup::ARM_2, "ARM_2"}};
        measured.at(JointGroup::ARM_1).tcp_pose = {0.4, 0.2, 0.2, 1, 0, 0, 0};
        measured.at(JointGroup::ARM_2).tcp_pose = {0.4, -0.2, 0.2, 1, 0, 0, 0};
        measured.at(JointGroup::ARM_1).q = {0.1, -0.7, 0.2, 1.5, -0.1, 0.7, 0.2};
        measured.at(JointGroup::ARM_2).q = std::vector<double>(7, 0);
    }
};
inline Data state;
inline void Require(bool condition, const std::string& message)
{
    if (!condition) throw std::runtime_error(message);
}
template <class Command>
void OnlyArm1(const std::map<JointGroup, Command>& commands)
{
    Require(commands.size() == 1 && commands.contains(JointGroup::ARM_1),
        "command addressed a group other than ARM_1");
}

inline void BeginDescent(const std::array<double, kPoseSize>& target)
{
    auto& s = state;
    Require(s.primitive_reads >= 4, "Z descent started before ARM_1 zeroing completed");
    s.descent_start = s.measured.at(JointGroup::ARM_1).tcp_pose;
    s.descent_start_q = s.measured.at(JointGroup::ARM_1).q;
    s.descent_target = target;
    if (s.fail_descent_plan) {
        s.fault = true;
        return;
    }
    s.descent_active = true;
    auto& measured = s.measured.at(JointGroup::ARM_1);
    measured.tcp_pose = target;
    measured.tcp_wrench = s.descent_wrench;
    measured.tcp_wrench_local = s.descent_local_wrench;
    if (s.contact_before_endpoint) {
        for (size_t i = 0; i < 3; ++i) {
            measured.tcp_pose[i] = s.descent_start[i] + 0.12 * (target[i] - s.descent_start[i]);
        }
    }
    if (s.descent_x_offset) measured.tcp_pose[0] += 0.002;
    if (s.descent_rotation_error) {
        measured.tcp_pose[3] = 1;
        measured.tcp_pose[4] = measured.tcp_pose[5] = measured.tcp_pose[6] = 0;
    }
    s.line_start = measured.tcp_pose;
}
} // namespace polishing_test

class Robot {
public:
    explicit Robot(const std::string&) { ++polishing_test::state.constructions; }
    RobotInfo info() const { return polishing_test::state.info; }
    bool fault() const { return polishing_test::state.fault; }
    bool operational() const { return polishing_test::state.ready; }
    bool all_stopped() const { return polishing_test::state.stopped; }
    std::map<JointGroup, bool> stopped() const
    {
        const auto& s = polishing_test::state;
        const bool moving = s.descent_active && s.descent_still_moving && s.descent_reads < 3;
        if (s.descent_active && s.missing_descent_stop && s.descent_reads < 3) {
            return {{JointGroup::ARM_2, true}};
        }
        return {{JointGroup::ARM_1, !moving}, {JointGroup::ARM_2, true}};
    }
    Mode mode() const { return polishing_test::state.mode; }
    OperationalStatus operational_status() const { return polishing_test::state.status; }
    std::map<JointGroup, RobotStates> states() const
    {
        auto& s = polishing_test::state;
        if (s.descent_active) {
            ++s.descent_reads;
            if ((s.contact_before_endpoint || s.descent_x_offset || s.descent_rotation_error)
                && s.descent_reads >= 3) {
                s.measured.at(JointGroup::ARM_1).tcp_pose = s.descent_target;
            }
            if (s.nonfinite_descent_pose) {
                s.measured.at(JointGroup::ARM_1).tcp_pose[2] = std::numeric_limits<double>::quiet_NaN();
            }
        }
        return s.measured;
    }
    std::vector<RobotEvent> event_log() const
    {
        auto& s = polishing_test::state;
        polishing_test::Require(s.stops > 0, "diagnostics delayed the Stop attempt");
        ++s.event_reads;
        if (s.event_reads <= s.event_delay_reads) return {};
        RobotEvent event;
        event.id = 4242;
        event.description = "injected trajectory failure";
        event.probable_causes = "test planner rejected the target";
        event.recommended_actions = "inspect the target";
        return {event};
    }
    void ServoOn()
    {
        auto& s = polishing_test::state;
        ++s.controls;
        ++s.servos;
        if (s.fail_servo) throw std::runtime_error("injected ServoOn failure");
        s.ready = true;
    }
    void Home(const std::vector<JointGroup>& groups = {})
    {
        polishing_test::Require(groups == std::vector<JointGroup>{JointGroup::ARM_1},
            "Home did not explicitly select ARM_1");
        auto& s = polishing_test::state;
        ++s.controls;
        ++s.homes;
        if (s.fail_home) throw std::runtime_error("injected Home failure");
    }
    void SwitchMode(Mode mode)
    {
        auto& s = polishing_test::state;
        ++s.controls;
        if (mode == Mode::RT_CARTESIAN_MOTION_FORCE) {
            polishing_test::Require(s.measured.at(JointGroup::ARM_1).tcp_pose == s.descent_target,
                "RT pressing started before completing the full 50 mm move");
            polishing_test::Require(!s.descent_still_moving || s.descent_reads >= 3,
                "RT pressing started before ARM_1 stopped at the position target");
            polishing_test::Require(s.descent_movels == 0 || s.descent_movel_reads >= 3,
                "RT pressing used another group's or primitive's completion");
            s.rt_entry_reads = s.descent_reads;
            s.descent_active = false;
            s.line_start = s.measured.at(JointGroup::ARM_1).tcp_pose;
        }
        s.mode = mode;
    }
    void SendCartesianMotionForce(const std::map<JointGroup, NrtCartesianCmd>& commands)
    {
        polishing_test::OnlyArm1(commands);
        auto& s = polishing_test::state;
        ++s.controls;
        ++s.sends;
        if (s.zeros == 0 && s.fail_nrt_plan) {
            s.fault = true;
            return; // Target rejected before any simulated motion.
        }
        if (s.zeros > 0) {
            polishing_test::Require(commands.at(JointGroup::ARM_1).max_linear_vel == 0.005,
                "descent speed is not 5 mm/s");
            polishing_test::Require(s.axis_settings == 1,
                "descent did not explicitly disable force control before moving");
            polishing_test::BeginDescent(commands.at(JointGroup::ARM_1).pose_d);
            return;
        }
        s.measured.at(JointGroup::ARM_1).tcp_pose = commands.at(JointGroup::ARM_1).pose_d;
        s.line_start = s.measured.at(JointGroup::ARM_1).tcp_pose;
    }
    void ExecutePrimitive(const std::map<JointGroup, PrimitiveArgs>& commands)
    {
        polishing_test::OnlyArm1(commands);
        const auto& p = commands.at(JointGroup::ARM_1);
        polishing_test::Require(!p.external_axis_control
                && p.sync_motion_mode == SyncMotionMode::DISABLE,
            "primitive enabled an external or synchronized group");
        auto& s = polishing_test::state;
        ++s.controls;
        s.active_primitive = p.pt_name;
        if (p.pt_name == "MoveL") {
            const bool descent = s.zeros > 0;
            if (descent) ++s.descent_movels;
            else ++s.movels;
            polishing_test::Require(std::get<double>(p.input_params.at("vel")) == (descent ? 0.005 : 0.01),
                "wrong MoveL linear velocity");
            polishing_test::Require(std::abs(std::get<double>(p.input_params.at("angVel"))
                    - 0.10 * 180 / std::acos(-1.0)) < 1e-12,
                "MoveL angular velocity was not converted to degrees/s");
            polishing_test::Require(std::get<double>(p.input_params.at("acc")) == 0.10
                    && std::get<std::string>(p.input_params.at("zoneRadius")) == "ZFine",
                "wrong MoveL acceleration or blending");
            const auto target = std::get<Coord>(p.input_params.at("target"));
            if (descent) s.descent_movel_target = target;
            else s.movel_target = target;
            if (!descent && s.fail_movel_plan) {
                s.fault = true;
                return;
            }
            polishing_test::Require(target.ref_frame == std::array<std::string, 2>{"WORLD", "WORLD_ORIGIN"},
                "MoveL target used the wrong frame");
            constexpr double degree = 3.14159265358979323846 / 180;
            const Eigen::Quaterniond rotation
                = Eigen::AngleAxisd(target.orientation[2] * degree, Eigen::Vector3d::UnitZ())
                * Eigen::AngleAxisd(target.orientation[1] * degree, Eigen::Vector3d::UnitY())
                * Eigen::AngleAxisd(target.orientation[0] * degree, Eigen::Vector3d::UnitX());
            const std::array<double, kPoseSize> pose = {target.position[0], target.position[1],
                target.position[2], rotation.w(), rotation.x(), rotation.y(), rotation.z()};
            if (descent) polishing_test::BeginDescent(pose);
            else s.measured.at(JointGroup::ARM_1).tcp_pose = pose;
        } else {
            polishing_test::Require(p.pt_name == "ZeroFTSensor", "unexpected primitive");
            s.after_tilt_pose = s.measured.at(JointGroup::ARM_1).tcp_pose;
            ++s.zeros;
        }
    }
    std::map<JointGroup, PrimitiveStates> primitive_states() const
    {
        auto& s = polishing_test::state;
        PrimitiveStates arm1, arm2;
        if (s.active_primitive == "MoveL") {
            const bool descent = s.zeros > 0;
            const int n = descent ? ++s.descent_movel_reads : ++s.movel_reads;
            const bool terminated = descent ? s.descent_movel_terminated : s.movel_terminated;
            arm1.pt_name = n == 2 ? "Home" : "MoveL";
            arm1.names_and_values = {{"reachedTarget", terminated ? 0 : 1},
                {"terminated", terminated ? 1 : 0}};
            arm2.pt_name = "MoveL";
            arm2.names_and_values = {{"reachedTarget", descent ? 1 : 0}};
            if (n == 1 && !terminated) return {{JointGroup::ARM_2, arm2}};
            return {{JointGroup::ARM_1, arm1}, {JointGroup::ARM_2, arm2}};
        }
        const int n = ++s.primitive_reads;
        if (n == 4 && s.bad_descent_joints) s.measured.at(JointGroup::ARM_1).q.clear();
        if (n == 4 && s.update_zero_joints) s.measured.at(JointGroup::ARM_1).q[0] += 0.2;
        if (n == 4 && s.alter_zero_orientation) {
            auto& pose = s.measured.at(JointGroup::ARM_1).tcp_pose;
            const Eigen::Quaterniond rotation = Eigen::AngleAxisd(0.005, Eigen::Vector3d::UnitX())
                * Eigen::Quaterniond(pose[3], pose[4], pose[5], pose[6]);
            pose[3] = rotation.w();
            pose[4] = rotation.x();
            pose[5] = rotation.y();
            pose[6] = rotation.z();
        }
        if (n == 1) return {};
        arm1.pt_name = arm2.pt_name = "ZeroFTSensor";
        arm1.names_and_values = {{"terminated", n >= 4 ? 1 : 0}};
        arm2.names_and_values = {{"terminated", n >= 4 ? 0 : 1}};
        if (n == 2) return {{JointGroup::ARM_2, arm2}};
        return {{JointGroup::ARM_1, arm1}, {JointGroup::ARM_2, arm2}};
    }
    void SetMaxContactWrench(JointGroup group, const std::array<double, kCartDoF>&)
    {
        polishing_test::Require(group == JointGroup::ARM_1, "wrench setting addressed ARM_2");
        ++polishing_test::state.controls;
        ++polishing_test::state.wrench_settings;
    }
    void SetForceControlFrame(JointGroup group, CoordType frame)
    {
        polishing_test::Require(group == JointGroup::ARM_1, "frame setting addressed ARM_2");
        ++polishing_test::state.controls;
        ++polishing_test::state.frame_settings;
        polishing_test::state.frame = frame;
    }
    void SetForceControlAxis(JointGroup group, const std::array<bool, kCartDoF>& axes)
    {
        polishing_test::Require(group == JointGroup::ARM_1, "axis setting addressed ARM_2");
        const bool rt = polishing_test::state.mode == Mode::RT_CARTESIAN_MOTION_FORCE;
        polishing_test::Require(!rt || polishing_test::state.frame == CoordType::TCP,
            "RT force axis must be expressed in the TCP frame");
        const std::array<bool, kCartDoF> expected = rt
            ? std::array<bool, kCartDoF>{false, false, true, false, false, false}
            : std::array<bool, kCartDoF>{};
        polishing_test::Require(axes == expected,
            "descent must position-control all axes; RT must force-control only Z");
        ++polishing_test::state.controls;
        ++polishing_test::state.axis_settings;
    }
    void StreamCartesianMotionForce(const std::map<JointGroup, RtCartesianCmd>& commands)
    {
        polishing_test::OnlyArm1(commands);
        auto& s = polishing_test::state;
        ++s.controls;
        ++s.streams;
        s.last_target = commands.at(JointGroup::ARM_1).pose_d;
        s.last_wrench = commands.at(JointGroup::ARM_1).wrench_d;
    }
    void Stop()
    {
        ++polishing_test::state.controls;
        ++polishing_test::state.stops;
        if (polishing_test::state.fail_stop) throw std::runtime_error("injected Stop failure");
    }
};
} // namespace flexiv::rdk
