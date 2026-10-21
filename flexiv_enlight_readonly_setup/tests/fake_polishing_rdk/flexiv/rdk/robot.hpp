#pragma once

// In-memory replacement: uses SDK data types, never links or loads the robot SDK.
#include <flexiv/rdk/data.hpp>
#include <flexiv/rdk/mode.hpp>
#include <Eigen/Geometry>
#include <functional>
#include <stdexcept>

namespace flexiv::rdk {
namespace polishing_test {
struct Data {
    RobotInfo info;
    std::map<JointGroup, RobotStates> measured {{JointGroup::ARM_1, {}}, {JointGroup::ARM_2, {}}};
    bool ready = true, fault = false, stopped = true, fail_home = false, fail_servo = false;
    bool fail_nrt_plan = false, fail_movel_plan = false, fail_stop = false;
    bool movel_terminated = false;
    Mode mode = Mode::IDLE;
    OperationalStatus status = OperationalStatus::READY;
    int constructions = 0, controls = 0, servos = 0, homes = 0, stops = 0;
    int sends = 0, streams = 0, zeros = 0, primitive_reads = 0;
    int movels = 0, movel_reads = 0, event_reads = 0;
    std::string active_primitive;
    Coord movel_target;
    int frame_settings = 0, axis_settings = 0, wrench_settings = 0, scheduler_stops = 0;
    CoordType frame = CoordType::WORLD;
    std::array<double, kPoseSize> line_start {}, last_target {};
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
} // namespace polishing_test

class Robot {
public:
    explicit Robot(const std::string&) { ++polishing_test::state.constructions; }
    RobotInfo info() const { return polishing_test::state.info; }
    bool fault() const { return polishing_test::state.fault; }
    bool operational() const { return polishing_test::state.ready; }
    bool all_stopped() const { return polishing_test::state.stopped; }
    Mode mode() const { return polishing_test::state.mode; }
    OperationalStatus operational_status() const { return polishing_test::state.status; }
    std::map<JointGroup, RobotStates> states() const { return polishing_test::state.measured; }
    std::vector<RobotEvent> event_log() const
    {
        auto& s = polishing_test::state;
        polishing_test::Require(s.stops > 0, "diagnostics delayed the Stop attempt");
        ++s.event_reads;
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
        ++polishing_test::state.controls;
        polishing_test::state.mode = mode;
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
            polishing_test::Require(s.primitive_reads >= 4,
                "contact search started before ARM_1 zeroing completed");
            s.measured.at(JointGroup::ARM_1).tcp_wrench[2] = 4;
        }
        s.measured.at(JointGroup::ARM_1).tcp_pose = commands.at(JointGroup::ARM_1).pose_d;
        s.line_start = commands.at(JointGroup::ARM_1).pose_d;
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
            ++s.movels;
            polishing_test::Require(std::get<double>(p.input_params.at("vel")) == 0.01,
                "wrong MoveL linear velocity");
            polishing_test::Require(std::abs(std::get<double>(p.input_params.at("angVel"))
                    - 0.10 * 180 / std::acos(-1.0)) < 1e-12,
                "MoveL angular velocity was not converted to degrees/s");
            polishing_test::Require(std::get<double>(p.input_params.at("acc")) == 0.10
                    && std::get<std::string>(p.input_params.at("zoneRadius")) == "ZFine",
                "wrong MoveL acceleration or blending");
            s.movel_target = std::get<Coord>(p.input_params.at("target"));
            if (s.fail_movel_plan) {
                s.fault = true;
                return;
            }
            const auto& target = s.movel_target;
            polishing_test::Require(target.ref_frame == std::array<std::string, 2>{"WORLD", "WORLD_ORIGIN"},
                "MoveL target used the wrong frame");
            constexpr double degree = 3.14159265358979323846 / 180;
            const Eigen::Quaterniond rotation
                = Eigen::AngleAxisd(target.orientation[2] * degree, Eigen::Vector3d::UnitZ())
                * Eigen::AngleAxisd(target.orientation[1] * degree, Eigen::Vector3d::UnitY())
                * Eigen::AngleAxisd(target.orientation[0] * degree, Eigen::Vector3d::UnitX());
            s.measured.at(JointGroup::ARM_1).tcp_pose = {target.position[0], target.position[1],
                target.position[2], rotation.w(), rotation.x(), rotation.y(), rotation.z()};
        } else {
            polishing_test::Require(p.pt_name == "ZeroFTSensor", "unexpected primitive");
            ++s.zeros;
        }
    }
    std::map<JointGroup, PrimitiveStates> primitive_states() const
    {
        auto& s = polishing_test::state;
        PrimitiveStates arm1, arm2;
        if (s.active_primitive == "MoveL") {
            const int n = ++s.movel_reads;
            arm1.pt_name = n == 2 ? "Home" : "MoveL";
            arm1.names_and_values = {{"reachedTarget", s.movel_terminated ? 0 : 1},
                {"terminated", s.movel_terminated ? 1 : 0}};
            arm2.pt_name = "MoveL";
            arm2.names_and_values = {{"reachedTarget", 0}};
            if (n == 1 && !s.movel_terminated) return {{JointGroup::ARM_2, arm2}};
            return {{JointGroup::ARM_1, arm1}, {JointGroup::ARM_2, arm2}};
        }
        const int n = ++s.primitive_reads;
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
    void SetForceControlAxis(JointGroup group, const std::array<bool, kCartDoF>&)
    {
        polishing_test::Require(group == JointGroup::ARM_1, "axis setting addressed ARM_2");
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
