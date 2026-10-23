#pragma once
// Uses vendor data definitions only. No SDK runtime is linked or loaded.
#include <flexiv/rdk/data.hpp>
#include <flexiv/rdk/mode.hpp>
#include <atomic>
#include <functional>
#include <stdexcept>

namespace flexiv::rdk::tilt_force_test {
// Main configures the force axis while the callback keeps streaming hold.
struct Counter {
    std::atomic<int> value {0};
    Counter() = default;
    Counter(const Counter& other) : value(other.value.load()) {}
    Counter& operator=(const Counter& other) { value.store(other.value.load()); return *this; }
    operator int() const { return value.load(); }
    void operator++() { value.fetch_add(1); }
};
struct Data {
    RobotInfo info;
    std::map<JointGroup, RobotStates> states {{JointGroup::ARM_1, {}}, {JointGroup::ARM_2, {}}};
    std::map<JointGroup, std::array<bool, 6>> axes;
    std::map<JointGroup, CoordType> frames;
    std::map<JointGroup, std::vector<double>> reference_q;
    std::array<double, 3> velocity {};
    std::vector<std::map<JointGroup, RtCartesianCmd>> packets;
    std::vector<std::map<JointGroup, RtJointTorqueCmd>> torque_packets;
    std::vector<std::map<JointGroup, PrimitiveArgs>> primitive_commands;
    std::vector<std::string> events;
    std::function<void()> after_stream, after_states, after_enable_force, after_home;
    std::function<void()> after_stop, after_stop_check, after_servo;
    bool connected = true, fault = false, ready = true, stopped = true;
    bool follow = true, contact = true, stamps = true, fail_stop = false, fail_stream = false;
    bool fail_home = false, home_never_reaches = false, home_terminated = false;
    bool hold_terminated = false, wrong_home_name = false, home_finished = false;
    bool home_reached_but_moving = false;
    bool stop_confirms = true, stop_enters_idle = true;
    int stop_reads = 0, servos = 0;
    std::array<double, 7> home_pose {0.4, 0.2, 0.12, 1, 0, 0, 0};
    std::vector<double> home_q = std::vector<double>(7, 0.1);
    int homes = 0, home_reads = 0;
    Mode mode = Mode::IDLE;
    int constructions = 0, controls = 0, stops = 0;
    Counter force_enables;
    Data()
    {
        for (auto arm : {JointGroup::ARM_1, JointGroup::ARM_2}) {
            info.single_arm_groups[arm] = arm == JointGroup::ARM_1 ? "ARM_1" : "ARM_2";
            info.K_x_nom[arm] = {1000, 1000, 1000, 100, 100, 100};
            states.at(arm).tcp_pose = {0.4, arm == JointGroup::ARM_1 ? 0.2 : -0.2, 0.12, 1, 0, 0, 0};
            states.at(arm).q = std::vector<double>(7, 0);
            states.at(arm).dtheta = std::vector<double>(7, 0);
        }
    }
};
inline Data data;
} // namespace flexiv::rdk::tilt_force_test

namespace flexiv::rdk {
class Robot {
    static void Write(const char* event)
    {
        ++tilt_force_test::data.controls;
        tilt_force_test::data.events.emplace_back(event);
    }
public:
    explicit Robot(const std::string&) { ++tilt_force_test::data.constructions; }
    auto info() const { return tilt_force_test::data.info; }
    bool connected() const { return tilt_force_test::data.connected; }
    bool fault() const { return tilt_force_test::data.fault; }
    bool operational() const { return tilt_force_test::data.ready; }
    bool all_stopped() const
    {
        auto& d = tilt_force_test::data;
        if (d.stops) {
            ++d.stop_reads;
            if (d.after_stop_check) d.after_stop_check();
        }
        return d.stopped;
    }
    auto mode() const { return tilt_force_test::data.mode; }
    auto states() const
    {
        auto& d = tilt_force_test::data;
        if (d.after_states) d.after_states();
        if (d.stamps) {
            for (auto& [_, state] : d.states) {
                state.timestamp.second += 1000000;
                if (state.timestamp.second >= 1000000000) {
                    state.timestamp.second -= 1000000000; ++state.timestamp.first;
                }
            }
        }
        return d.states;
    }
    void SetPassiveForceControl(JointGroup group, bool enabled)
    {
        if (group != JointGroup::ARM_1 || enabled || mode() != Mode::IDLE)
            throw std::runtime_error("incorrect active-force setup");
        Write("active-force");
    }
    void SwitchMode(Mode mode) { Write("mode"); tilt_force_test::data.mode = mode; }
    void SetForceControlAxis(JointGroup group, const std::array<bool, 6>& axes,
        const std::array<double, 3>& velocity = {1, 1, 1})
    {
        Write("axes");
        auto& d = tilt_force_test::data;
        d.axes[group] = axes;
        if (axes[2]) {
            ++d.force_enables; d.velocity = velocity;
            if (d.after_enable_force) d.after_enable_force();
        }
    }
    void SetCartesianImpedance(JointGroup, const std::array<double, 6>&) { Write("stiffness"); }
    void SetMaxContactWrench(JointGroup, const std::array<double, 6>&) { Write("wrench-limit"); }
    void SetNullSpacePosture(JointGroup group, const std::vector<double>& q)
    { Write("reference"); tilt_force_test::data.reference_q[group] = q; }
    void SetNullSpaceObjectives(JointGroup group, double, double, double weight)
    {
        if (group != JointGroup::ARM_2 || weight != 1) throw std::runtime_error("bad hold objective");
        Write("hold-objective");
    }
    void SetForceControlFrame(JointGroup group, CoordType frame)
    { Write("frame"); tilt_force_test::data.frames[group] = frame; }
    void ExecutePrimitive(const std::map<JointGroup, PrimitiveArgs>& commands, bool wait_started)
    {
        auto& d = tilt_force_test::data;
        Write("home-start");
        if (d.mode != Mode::NRT_PRIMITIVE_EXECUTION || !wait_started || commands.size() != 2
            || commands.at(JointGroup::ARM_1).pt_name != "Home"
            || commands.at(JointGroup::ARM_2).pt_name != "Hold")
            throw std::runtime_error("incorrect startup Home / Hold command");
        d.primitive_commands.push_back(commands); ++d.homes;
        if (d.fail_home) throw std::runtime_error("injected Home failure");
        d.stopped = false;
    }
    std::map<JointGroup, PrimitiveStates> primitive_states()
    {
        auto& d = tilt_force_test::data;
        ++d.home_reads;
        const bool reached = !d.home_never_reaches && !d.home_terminated && d.home_reads >= 3;
        if (reached && !d.home_finished) {
            d.home_finished = true; d.events.emplace_back("home-complete");
            d.states.at(JointGroup::ARM_1).tcp_pose = d.home_pose;
            d.states.at(JointGroup::ARM_1).q = d.home_q;
            d.stopped = !d.home_reached_but_moving;
            if (d.after_home) d.after_home();
        }
        return {{JointGroup::ARM_1, {d.wrong_home_name ? "MoveL" : "Home",
                    {{"reachedTarget", int(reached)}, {"terminated", int(d.home_terminated)}}}},
            {JointGroup::ARM_2, {"Hold", {{"terminated", int(d.hold_terminated)}}}}};
    }
    void StreamCartesianMotionForce(const std::map<JointGroup, RtCartesianCmd>& packets)
    {
        auto& d = tilt_force_test::data;
        if (d.fail_stream) throw std::runtime_error("injected stream failure");
        if (!d.home_finished) throw std::runtime_error("RT stream preceded Home completion");
        d.packets.push_back(packets); d.stopped = false;
        if (d.follow) for (const auto& [group, cmd] : packets) d.states.at(group).tcp_pose = cmd.pose_d;
        if (d.contact && d.force_enables)
            d.states.at(JointGroup::ARM_1).tcp_wrench_local = packets.at(JointGroup::ARM_1).wrench_d;
        if (d.after_stream) d.after_stream();
    }
    bool ClearFault() { Write("clear-fault"); tilt_force_test::data.fault = false; return true; }
    void ServoOn()
    {
        Write("servo"); auto& d = tilt_force_test::data; ++d.servos; d.ready = true;
        if (d.after_servo) d.after_servo();
    }
    void Home()
    {
        Write("floating-home"); auto& d = tilt_force_test::data;
        ++d.homes; d.home_finished = true;
        if (d.fail_home) throw std::runtime_error("injected Home failure");
        if (d.after_home) d.after_home();
    }
    void StreamJointTorque(const std::map<JointGroup, RtJointTorqueCmd>& packets)
    {
        auto& d = tilt_force_test::data;
        if (d.fail_stream) throw std::runtime_error("injected stream failure");
        if (d.mode != Mode::RT_JOINT_TORQUE || d.stops)
            throw std::runtime_error("torque streamed outside active control");
        d.torque_packets.push_back(packets); d.stopped = false;
        if (d.after_stream) d.after_stream();
    }
    void Stop()
    {
        Write("stop"); ++tilt_force_test::data.stops;
        if (tilt_force_test::data.fail_stop) throw std::runtime_error("injected Stop failure");
        auto& d = tilt_force_test::data;
        d.stopped = d.stop_confirms;
        if (d.stop_enters_idle) d.mode = Mode::IDLE;
        if (d.after_stop) d.after_stop();
    }
    std::vector<RobotEvent> event_log() const { return {}; }
};
} // namespace flexiv::rdk
