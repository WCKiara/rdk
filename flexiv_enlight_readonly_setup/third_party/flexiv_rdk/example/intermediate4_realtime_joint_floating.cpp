/**
 * @example intermediate4_realtime_joint_floating.cpp
 * This tutorial runs real-time joint floating with gentle velocity damping, gravity compensation,
 * and soft protection against position limits. This example is ideal for verifying the system's
 * whole-loop real-timeliness, accuracy of the robot dynamics model, and joint torque control
 * performance. If everything works well, all joints should float smoothly.
 * @copyright Copyright (C) 2016-2026 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <flexiv/rdk/robot.hpp>
#include <flexiv/rdk/scheduler.hpp>
#include <flexiv/rdk/utility.hpp>
#include "stop_and_confirm.hpp"

#include <csignal>
#include <exception>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <atomic>

using namespace flexiv;

namespace {
/** Joint velocity damping gains [Nm*s/rad] for this RT_JOINT_TORQUE example only.
 * Does not configure Elements Manual/Auto freedrive or the FloatingJoint primitive. */
const std::vector<double> kFloatingDamping = {2.5, 2.5, 1.25, 1.25, 0.25, 0.25, 0.25};

/** Atomic signal to stop scheduler tasks */
std::atomic<bool> g_stop_sched = {false};
std::atomic<bool> g_task_failed = {false};
std::atomic<int> g_signal {0};
static_assert(std::atomic<int>::is_always_lock_free);

void OnSignal(int signum) { g_signal.store(signum); }
void CheckInterrupted()
{
    if (g_signal.load()) throw std::runtime_error("Floating interrupted");
}
}

/** @brief Print program usage help */
void PrintHelp()
{
    // clang-format off
    std::cout << "Required arguments: [robot_sn]" << std::endl;
    std::cout << "    robot_sn: Serial number of the robot to connect. Remove any space, e.g. Enlight-L-123456" << std::endl;
    std::cout << "Optional arguments: None" << std::endl;
    std::cout << "Ctrl+C: stop floating and confirm IDLE position hold before exit." << std::endl;
    std::cout << std::endl;
    // clang-format on
}

/** @brief Callback function for realtime periodic task */
void PeriodicTask(rdk::Robot& robot, const std::map<rdk::JointGroup, std::string>& exe_groups)
{
    if (g_stop_sched.load() || g_signal.load()) return;
    try {
        // Monitor fault on the connected robot
        if (robot.fault()) {
            throw std::runtime_error(
                "PeriodicTask: Fault occurred on the connected robot, exiting ...");
        }

        std::map<rdk::JointGroup, rdk::RtJointTorqueCmd> rt_cmds;
        const auto all_states = robot.states();
        for (const auto& [group, _] : exe_groups) {
            const auto& states = all_states.at(group);
            std::vector<double> target_torque(states.q.size());

            // Don't add damping for external axis
            if (group != rdk::JointGroup::EXT_AXIS) {
                if (target_torque.size() != kFloatingDamping.size()
                    || states.dtheta.size() != kFloatingDamping.size())
                    throw std::runtime_error("Floating requires seven valid joint velocities");
                // Add some velocity damping
                for (size_t i = 0; i < target_torque.size(); ++i) {
                    if (!std::isfinite(states.dtheta[i]))
                        throw std::runtime_error("Non-finite floating joint velocity");
                    target_torque[i] += -kFloatingDamping[i] * states.dtheta[i];
                }
            }

            rt_cmds[group] = rdk::RtJointTorqueCmd(target_torque, true, true);
        }

        // Send target joint torque to RDK server, enable gravity compensation and joint limits soft
        // protection
        robot.StreamJointTorque(rt_cmds);

    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
        g_task_failed = true;
        g_stop_sched = true;
    }
}

int RunFloating(const std::string& robot_sn)
{
    // Print description
    std::cout <<
        ">>> Tutorial description <<<\nThis tutorial runs real-time joint floating with gentle "
        "velocity damping, gravity compensation, and soft protection against position limits. This "
        "example is ideal for verifying the system's whole-loop real-timeliness, accuracy of the "
        "robot dynamics model, and joint torque control performance. If everything works well, all "
        "joints should float smoothly.\n";

    rdk::Robot robot(robot_sn);
    std::unique_ptr<rdk::Scheduler> scheduler;
    bool control_started = false, scheduler_started = false;
    std::exception_ptr error;
    try {
        CheckInterrupted();

        // Clear fault on the connected robot if any
        if (robot.fault()) {
            std::cerr << "Fault occurred on the connected robot, trying to clear ...\n";
            // Try to clear the fault
            if (!robot.ClearFault()) {
                throw std::runtime_error("Fault cannot be cleared, exiting ...");
            }
            std::cout << "Fault on the connected robot is cleared\n";
        }

        // Servo on the robot, make sure the E-stop is released
        CheckInterrupted();
        std::cout << "Servo on the robot ...\n";
        control_started = true;
        robot.ServoOn();

        // Wait for the robot to become operational
        while (!robot.operational()) {
            CheckInterrupted();
            if (robot.fault()) throw std::runtime_error("Robot fault during startup");
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        std::cout << "Robot is now operational\n";

        // Move robot to home pose
        CheckInterrupted();
        std::cout << "Moving to home pose\n";
        robot.Home();

        // Real-time Joint Floating
        // =========================================================================================
        // Direct joint control can be executed by single-arm joint groups and the external axis
        auto exe_groups = robot.info().single_arm_groups;
        if (exe_groups.empty()) {
            throw std::runtime_error("No single-arm joint group found on the connected robot");
        }
        // The external axis joint group (if exists) also supports direct joint control
        if (robot.info().all_groups.contains(rdk::JointGroup::EXT_AXIS)) {
            exe_groups.emplace(
                rdk::JointGroup::EXT_AXIS, robot.info().all_groups.at(rdk::JointGroup::EXT_AXIS));
        }

        // Switch to real-time joint torque control mode
        CheckInterrupted();
        robot.SwitchMode(rdk::Mode::RT_JOINT_TORQUE);

        // Create real-time scheduler to run periodic tasks
        scheduler = std::make_unique<rdk::Scheduler>();
        // Add periodic task with 1ms interval and highest applicable priority
        scheduler->AddTask([&robot, exe_groups] { PeriodicTask(robot, exe_groups); },
            "HP periodic", 1, scheduler->max_priority());
        // Start all added tasks
        scheduler->Start();
        scheduler_started = true;

        // Block and wait for signal to stop scheduler tasks
        while (!g_stop_sched.load() && !g_signal.load()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        if (g_task_failed.load()) throw std::runtime_error("Floating callback failed");
    } catch (...) {
        error = std::current_exception();
    }

    g_stop_sched = true;
    if (scheduler_started) {
        try { scheduler->Stop(); }
        catch (...) { if (!error) error = std::current_exception(); }
    }
    scheduler.reset(); // Join any remaining callback before the robot's Stop().
    if (control_started) {
        try {
            example::StopAndConfirmIdle(robot);
            std::cout << "STOP_CONFIRMED mode=IDLE all_stopped=1 stable_seconds=0.2\n";
        } catch (const std::exception& stop_error) {
            std::cerr << "STOP_UNCONFIRMED: " << stop_error.what() << '\n';
            if (!error) error = std::current_exception();
        }
    }
    if (error) std::rethrow_exception(error);
    return g_signal.load() ? 128 + g_signal.load() : 0;
}

int main(int argc, char* argv[])
{
    if (argc < 2 || rdk::utility::ProgramArgsExistAny(argc, argv, {"-h", "--help"})) {
        PrintHelp();
        return argc < 2 ? 1 : 0;
    }
    g_stop_sched = false;
    g_task_failed = false;
    g_signal = 0;
    std::signal(SIGINT, OnSignal);
    std::signal(SIGTERM, OnSignal);
    std::signal(SIGHUP, OnSignal);
    try {
        return RunFloating(argv[1]);
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return g_signal.load() ? 128 + g_signal.load() : 1;
    }
}
