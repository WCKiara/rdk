#pragma once

// Deliberately minimal, in-memory SDK replacement for the lifecycle test.
// This header never includes or loads Flexiv's SDK and cannot contact hardware.
#include <array>
#include <chrono>
#include <cstdint>
#include <functional>
#include <map>
#include <string>
#include <utility>
#include <variant>
#include <vector>

namespace flexiv::rdk {
enum class JointGroup { ARM_1, ARM_2 };
enum class Mode { IDLE, RT_CARTESIAN_MOTION_FORCE, NRT_PRIMITIVE_EXECUTION };
enum class OperationalStatus { UNKNOWN, READY, BOOTING, ESTOP_NOT_RELEASED, NOT_SERVO_ON,
    RELEASING_BRAKE, MINOR_FAULT, CRITICAL_FAULT, IN_REDUCED_STATE, IN_RECOVERY_STATE,
    IN_MANUAL_MODE, IN_AUTO_MODE };
inline const std::map<OperationalStatus, std::string> kOpStatusNames {
    {OperationalStatus::UNKNOWN, "Unknown status"}, {OperationalStatus::READY, "Ready"},
    {OperationalStatus::BOOTING, "System booting"},
    {OperationalStatus::ESTOP_NOT_RELEASED, "E-Stop not released"},
    {OperationalStatus::NOT_SERVO_ON, "Not servo on"},
    {OperationalStatus::RELEASING_BRAKE, "Releasing brake"},
    {OperationalStatus::MINOR_FAULT, "Minor fault occurred"},
    {OperationalStatus::CRITICAL_FAULT, "Critical fault occurred"},
    {OperationalStatus::IN_REDUCED_STATE, "In reduced state"},
    {OperationalStatus::IN_RECOVERY_STATE, "In recovery state"},
    {OperationalStatus::IN_MANUAL_MODE, "In Manual mode"},
    {OperationalStatus::IN_AUTO_MODE, "In regular Auto mode"}};
enum class SyncMotionMode { ENABLE, DISABLE };
using Pose = std::array<double, 7>;

struct RobotStates {
    std::vector<double> q = std::vector<double>(7, 0);
    std::vector<double> dq = std::vector<double>(7, 0);
    Pose tcp_pose {0.4, 0.2, 0.12, 1, 0, 0, 0};
    std::array<double, 6> tcp_wrench {};
    std::pair<int, int> timestamp {0, 0}; // Match the SDK's signed seconds/nanoseconds.
};
struct RobotActions {
    std::pair<int, int> timestamp {0, 0};
    Pose tcp_pose_d {0.4, 0.2, 0.12, 1, 0, 0, 0};
};
struct RobotEvent {
    enum Level { UNKNOWN = 0, INFO, WARNING, ERROR, CRITICAL };
    Level level = UNKNOWN;
    int id = 0;
    std::string description, consequences, probable_causes, recommended_actions;
    std::chrono::time_point<std::chrono::system_clock> timestamp;
};
struct RobotInfo {
    std::map<JointGroup, int> single_arm_groups {{JointGroup::ARM_1, 7}, {JointGroup::ARM_2, 7}};
};
struct RtCartesianCmd {
    Pose pose_d;
    std::array<double, 6> wrench_d {};
    explicit RtCartesianCmd(const Pose& pose) : pose_d(pose) {}
};
struct PrimitiveArgs {
    std::string name;
    bool external_axis_control = true;
    SyncMotionMode sync_motion_mode = SyncMotionMode::ENABLE;
    PrimitiveArgs(std::string primitive, std::map<std::string, std::variant<int, double, std::string>>)
    : name(std::move(primitive)) {}
};
struct PrimitiveStates {
    std::string pt_name;
    std::map<std::string, std::variant<int, double, std::string>> names_and_values;
};

struct FakeRobotData {
    int constructions = 0, streams = 0, stops = 0, homes = 0, fault_clears = 0;
    int state_reads = 0, first_control_state_reads = 0;
    int action_reads = 0, event_log_reads = 0, event_log_reads_before_stop = 0;
    std::uint64_t first_control_arm2_time_ns = 0;
    bool connected = true, fault = false, operational = true, stopped = true;
    bool follow_commands = true, home_reached = true, stop_confirms = true;
    bool advance_arm1_stamp = true, advance_arm2_stamp = true;
    Mode mode = Mode::IDLE;
    OperationalStatus status = OperationalStatus::READY;
    RobotInfo info;
    std::map<JointGroup, RobotStates> states {{JointGroup::ARM_1, {}}, {JointGroup::ARM_2, {}}};
    std::map<JointGroup, RobotActions> actions {{JointGroup::ARM_1, {}}, {JointGroup::ARM_2, {}}};
    std::vector<RobotEvent> event_log;
    std::vector<std::string> events;
    std::vector<std::map<JointGroup, RtCartesianCmd>> stream_commands;
    std::vector<std::map<JointGroup, PrimitiveArgs>> primitives;
    std::vector<JointGroup> axis_groups, wrench_groups;
    std::array<bool, 6> force_axes {true, true, true, true, true, true};
    std::function<void()> after_stream, after_stop, after_servo, after_states;
    std::function<void(Mode)> after_switch_mode;
};
inline FakeRobotData fake;

class Robot {
public:
    explicit Robot(const std::string&)
    {
        ++fake.constructions;
        fake.events.push_back("construct");
    }
    RobotInfo info() const { return fake.info; }
    bool connected() const { return fake.connected; }
    bool fault() const { return fake.fault; }
    bool operational() const { return fake.operational; }
    bool all_stopped() const { return fake.stopped; }
    Mode mode() const { return fake.mode; }
    OperationalStatus operational_status() const { return fake.status; }
    std::map<JointGroup, RobotActions> actions() const
    {
        ++fake.action_reads;
        return fake.actions;
    }
    std::vector<RobotEvent> event_log() const
    {
        ++fake.event_log_reads;
        if (fake.stops == 0) ++fake.event_log_reads_before_stop;
        return fake.event_log;
    }
    std::map<JointGroup, RobotStates> states() const
    {
        ++fake.state_reads;
        for (auto& [group, state] : fake.states) {
            if ((group == JointGroup::ARM_1 && fake.advance_arm1_stamp)
                || (group == JointGroup::ARM_2 && fake.advance_arm2_stamp)) {
                state.timestamp.second += 1000000; // One millisecond per fresh fake sample.
                if (state.timestamp.second >= 1000000000) {
                    state.timestamp.second -= 1000000000;
                    ++state.timestamp.first;
                }
            }
        }
        if (fake.after_states) fake.after_states();
        return fake.states;
    }
    bool ClearFault(unsigned int)
    {
        ++fake.fault_clears;
        fake.events.push_back("clear-fault");
        fake.fault = false;
        fake.status = fake.operational ? OperationalStatus::READY : OperationalStatus::NOT_SERVO_ON;
        return true;
    }
    void ServoOn()
    {
        fake.events.push_back("servo");
        fake.operational = true;
        fake.status = OperationalStatus::READY;
        if (fake.after_servo) fake.after_servo();
    }
    void SwitchMode(Mode mode)
    {
        fake.events.push_back("mode");
        if (fake.first_control_state_reads == 0) {
            fake.first_control_state_reads = fake.state_reads;
            const auto stamp = fake.states.at(JointGroup::ARM_2).timestamp;
            fake.first_control_arm2_time_ns = static_cast<std::uint64_t>(stamp.first) * 1000000000 + stamp.second;
        }
        fake.mode = mode;
        fake.events.push_back(mode == Mode::RT_CARTESIAN_MOTION_FORCE ? "mode:rt" : "mode:primitive");
        if (fake.after_switch_mode) fake.after_switch_mode(mode);
    }
    void SetForceControlAxis(JointGroup group, const std::array<bool, 6>& axes)
    {
        fake.axis_groups.push_back(group);
        fake.force_axes = axes;
    }
    void SetMaxContactWrench(JointGroup group, const std::array<double, 6>&)
    {
        fake.wrench_groups.push_back(group);
    }
    void StreamCartesianMotionForce(const std::map<JointGroup, RtCartesianCmd>& commands)
    {
        ++fake.streams;
        fake.events.push_back("stream");
        fake.stream_commands.push_back(commands);
        fake.stopped = false;
        for (const auto& [group, command] : commands) {
            fake.actions.at(group).tcp_pose_d = command.pose_d;
            fake.actions.at(group).timestamp = fake.states.at(group).timestamp;
        }
        if (fake.follow_commands) {
            for (const auto& [group, command] : commands) fake.states.at(group).tcp_pose = command.pose_d;
        }
        if (fake.after_stream) fake.after_stream();
    }
    void ExecutePrimitive(const std::map<JointGroup, PrimitiveArgs>& commands, bool)
    {
        fake.primitives.push_back(commands);
        for (const auto& [group, primitive] : commands) {
            if (primitive.name == "Home") {
                ++fake.homes;
                fake.events.push_back("home");
                fake.states.at(group).tcp_pose = Pose {0.4, 0.2, 0.12, 1, 0, 0, 0};
            }
        }
        fake.stopped = false;
    }
    std::map<JointGroup, PrimitiveStates> primitive_states() const
    {
        return {{JointGroup::ARM_1,
            {"Home", {{"reachedTarget", fake.home_reached ? 1 : 0}, {"terminated", 1}}}}};
    }
    void Stop()
    {
        ++fake.stops;
        fake.events.push_back("stop");
        if (fake.stop_confirms) {
            fake.mode = Mode::IDLE;
            fake.stopped = true;
        }
        if (fake.after_stop) fake.after_stop();
    }
};
} // namespace flexiv::rdk
