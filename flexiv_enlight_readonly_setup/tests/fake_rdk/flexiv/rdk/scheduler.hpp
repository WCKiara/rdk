#pragma once

// Ordinary local thread only; no real-time privileges and no hardware.
#include <atomic>
#include <chrono>
#include <functional>
#include <stdexcept>
#include <string>
#include <thread>

namespace flexiv::rdk {
inline std::atomic<bool> fake_scheduler_skip_callbacks {false};
inline std::atomic<int> fake_scheduler_pause_callbacks_ms {0};
inline std::atomic<unsigned int> fake_scheduler_callbacks {0};
inline std::atomic<bool> fake_scheduler_throw_on_stop {false};
inline std::atomic<bool> fake_scheduler_joined {false};
inline std::atomic<bool> fake_scheduler_destroyed {false};
inline int fake_scheduler_constructor_delay_ms = 0;
inline int fake_scheduler_start_delay_ms = 0;
inline int fake_scheduler_first_callback_delay_ms = 0;
inline bool fake_scheduler_throw_in_constructor = false;
inline bool fake_scheduler_throw_on_start = false;
inline std::function<void()> fake_scheduler_after_constructor;
inline std::function<void()> fake_scheduler_before_start;
inline std::function<void()> fake_scheduler_before_first_callback;

inline void ResetFakeScheduler()
{
    fake_scheduler_skip_callbacks.store(false);
    fake_scheduler_pause_callbacks_ms.store(0);
    fake_scheduler_callbacks.store(0);
    fake_scheduler_throw_on_stop.store(false);
    fake_scheduler_joined.store(false);
    fake_scheduler_destroyed.store(false);
    fake_scheduler_constructor_delay_ms = 0;
    fake_scheduler_start_delay_ms = 0;
    fake_scheduler_first_callback_delay_ms = 0;
    fake_scheduler_throw_in_constructor = false;
    fake_scheduler_throw_on_start = false;
    fake_scheduler_after_constructor = {};
    fake_scheduler_before_start = {};
    fake_scheduler_before_first_callback = {};
}

class Scheduler {
    std::function<void()> task_;
    std::atomic<bool> running_ {false};
    std::thread worker_;
    void WaitWhileRunning(int milliseconds)
    {
        for (int i = 0; i < milliseconds && running_.load(); ++i)
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    void JoinTasks()
    {
        running_.store(false);
        if (worker_.joinable()) worker_.join();
        fake_scheduler_joined.store(true);
    }
public:
    Scheduler()
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(fake_scheduler_constructor_delay_ms));
        if (fake_scheduler_throw_in_constructor)
            throw std::runtime_error("injected scheduler constructor failure");
        if (fake_scheduler_after_constructor) fake_scheduler_after_constructor();
    }
    int max_priority() const { return 0; }
    template <class Callback>
    void AddTask(Callback callback, const std::string&, int, int) { task_ = callback; }
    void Start()
    {
        if (fake_scheduler_before_start) fake_scheduler_before_start();
        std::this_thread::sleep_for(std::chrono::milliseconds(fake_scheduler_start_delay_ms));
        if (fake_scheduler_throw_on_start)
            throw std::runtime_error("injected scheduler Start failure");
        running_.store(true);
        worker_ = std::thread([this]() {
            WaitWhileRunning(fake_scheduler_first_callback_delay_ms);
            while (running_.load()) {
                if (!fake_scheduler_skip_callbacks.load()) {
                    if (fake_scheduler_callbacks.fetch_add(1) == 0
                        && fake_scheduler_before_first_callback)
                        fake_scheduler_before_first_callback();
                    task_();
                }
                WaitWhileRunning(fake_scheduler_pause_callbacks_ms.exchange(0));
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        });
    }
    void Stop()
    {
        // A failed Stop may leave a worker alive; the destructor must still join.
        if (fake_scheduler_throw_on_stop.exchange(false))
            throw std::runtime_error("injected scheduler Stop failure");
        JoinTasks();
    }
    ~Scheduler() noexcept
    {
        JoinTasks();
        fake_scheduler_destroyed.store(true);
    }
};
} // namespace flexiv::rdk
