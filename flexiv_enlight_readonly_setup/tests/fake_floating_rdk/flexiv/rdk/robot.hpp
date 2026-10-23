#pragma once
// In-memory primitive commands only; no robot SDK runtime or network.
#include <flexiv/rdk/data.hpp>
#include <flexiv/rdk/mode.hpp>
#include <functional>
#include <stdexcept>

namespace flexiv::rdk::floating_test {
struct Data {
    RobotInfo info;
    std::map<JointGroup, RobotStates> states {{JointGroup::ARM_1, {}}, {JointGroup::ARM_2, {}}};
    std::vector<std::map<JointGroup, PrimitiveArgs>> commands;
    std::function<void()> on_home_read, on_floating_read;
    Mode mode = Mode::IDLE;
    OperationalStatus status = OperationalStatus::READY;
    bool connected = true, fault = false, ready = true, stopped = true;
    bool fail_home = false, home_never_reaches = false, home_terminated = false;
    bool hold_terminated = false, hold_missing = false, floating_terminated = false;
    bool fail_floating = false, fail_stop = false, stop_confirms = true, require_home = true;
    int controls = 0, homes = 0, floats = 0, stops = 0, servos = 0;
    int home_reads = 0, floating_reads = 0;
    std::string active;
    Data()
    {
        for (auto arm : {JointGroup::ARM_1, JointGroup::ARM_2}) {
            info.single_arm_groups[arm] = arm == JointGroup::ARM_1 ? "ARM_1" : "ARM_2";
            states.at(arm).q = std::vector<double>(7, 0);
        }
    }
};
inline Data data;
} // namespace flexiv::rdk::floating_test

namespace flexiv::rdk {
class Robot {
public:
    explicit Robot(const std::string&) {}
    auto info() const { return floating_test::data.info; }
    auto states() const { return floating_test::data.states; }
    bool connected() const { return floating_test::data.connected; }
    bool fault() const { return floating_test::data.fault; }
    bool operational() const { return floating_test::data.ready; }
    bool all_stopped() const { return floating_test::data.stopped; }
    auto mode() const { return floating_test::data.mode; }
    auto operational_status() const { return floating_test::data.status; }
    void ServoOn()
    {
        auto& d = floating_test::data; ++d.controls; ++d.servos;
        d.ready = true; d.status = OperationalStatus::READY;
    }
    void SwitchMode(Mode mode)
    {
        if (mode != Mode::NRT_PRIMITIVE_EXECUTION)
            throw std::runtime_error("Unexpected mode switch");
        ++floating_test::data.controls; floating_test::data.mode = mode;
    }
    void ExecutePrimitive(const std::map<JointGroup, PrimitiveArgs>& commands, bool wait_started)
    {
        auto& d = floating_test::data;
        if (d.mode != Mode::NRT_PRIMITIVE_EXECUTION || !wait_started)
            throw std::runtime_error("Wrong execution mode or wait option");
        ++d.controls; d.commands.push_back(commands);
        if (commands.size() != 2 || commands.at(JointGroup::ARM_2).pt_name != "Hold")
            throw std::runtime_error("ARM_2 Hold was not paired");
        for (const auto& [_, command] : commands)
            if (command.external_axis_control || command.sync_motion_mode != SyncMotionMode::DISABLE)
                throw std::runtime_error("External or synchronized motion enabled");
        d.active = commands.at(JointGroup::ARM_1).pt_name;
        if (d.active == "Home") {
            ++d.homes;
            if (d.fail_home) throw std::runtime_error("injected Home failure");
        } else if (d.active == "FloatingJoint") {
            if (!d.stopped || (d.require_home && (d.homes != 1 || d.home_reads < 2)))
                throw std::runtime_error("Floating preceded stopped Home arrival");
            ++d.floats;
            if (d.fail_floating) throw std::runtime_error("injected FloatingJoint failure");
        } else throw std::runtime_error("Unexpected ARM_1 primitive");
        d.stopped = false;
    }
    std::map<JointGroup, PrimitiveStates> primitive_states()
    {
        auto& d = floating_test::data;
        bool reached = false, terminated = false;
        if (d.active == "Home") {
            ++d.home_reads;
            reached = !d.home_never_reaches && !d.home_terminated && d.home_reads >= 2;
            terminated = d.home_terminated;
            if (reached) d.stopped = true;
            if (d.on_home_read) d.on_home_read();
        } else {
            ++d.floating_reads; terminated = d.floating_terminated;
            if (d.on_floating_read) d.on_floating_read();
        }
        std::map<JointGroup, PrimitiveStates> result {
            {JointGroup::ARM_1, {d.active, {{"reachedTarget", int(reached)}, {"terminated", int(terminated)}}}}};
        if (!d.hold_missing)
            result[JointGroup::ARM_2] = {"Hold", {{"terminated", int(d.hold_terminated)}}};
        return result;
    }
    void Stop()
    {
        auto& d = floating_test::data; ++d.controls; ++d.stops;
        if (d.fail_stop) throw std::runtime_error("injected Stop failure");
        d.mode = Mode::IDLE; d.stopped = d.stop_confirms;
    }
};
} // namespace flexiv::rdk
