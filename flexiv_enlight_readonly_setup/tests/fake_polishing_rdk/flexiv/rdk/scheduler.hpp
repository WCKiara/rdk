#pragma once

#include <flexiv/rdk/robot.hpp>

namespace flexiv::rdk {
// Synchronous callbacks advance the example's simulated time; no RT threads.
class Scheduler {
    std::function<void()> task_;
public:
    int max_priority() const { return 0; }
    template <class Callback>
    void AddTask(Callback callback, const std::string&, int, int) { task_ = callback; }
    void Start()
    {
        auto& s = polishing_test::state;
        for (int i = 0; i < 10000 && !s.stop_requested(); ++i) task_();
        polishing_test::Require(s.stop_requested(), "example did not finish within 10000 callbacks");
    }
    void Stop() { ++polishing_test::state.scheduler_stops; }
};
} // namespace flexiv::rdk
