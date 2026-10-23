#pragma once

#include <flexiv/rdk/robot.hpp>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <thread>

namespace example {
// Call only after all command callbacks have exited. Stop() performs the
// controller's stop and enters IDLE, whose documented behavior is position hold.
// The timeout below covers confirmation after the blocking SDK Stop() returns.
inline void StopAndConfirmIdle(flexiv::rdk::Robot& robot, double timeout = 3.0)
{
    constexpr double kStableSeconds = 0.2;
    if (!std::isfinite(timeout) || timeout <= kStableSeconds)
        throw std::invalid_argument("Stop confirmation timeout must exceed 0.2 seconds");
    robot.Stop();
    using Clock = std::chrono::steady_clock;
    const auto begin = Clock::now();
    auto stable_since = begin;
    bool stable = false;
    while (true) {
        if (!robot.connected()) throw std::runtime_error("Disconnected during stop confirmation");
        if (robot.fault()) throw std::runtime_error("Robot fault during stop confirmation");
        const auto now = Clock::now();
        if (robot.operational() && robot.mode() == flexiv::rdk::Mode::IDLE && robot.all_stopped()) {
            if (!stable) { stable = true; stable_since = now; }
            if (std::chrono::duration<double>(now - stable_since).count() >= kStableSeconds) return;
        } else {
            stable = false;
        }
        if (std::chrono::duration<double>(now - begin).count() >= timeout)
            throw std::runtime_error("Stop confirmation timeout: READY, IDLE and all_stopped required");
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}
} // namespace example
