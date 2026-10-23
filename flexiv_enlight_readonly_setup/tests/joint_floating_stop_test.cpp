// In-memory Robot and ordinary-thread Scheduler; no SDK runtime or hardware.
#define main FloatingMain
#include "../third_party/flexiv_rdk/example/intermediate4_realtime_joint_floating.cpp"
#undef main

namespace {
auto& fake = flexiv::rdk::tilt_force_test::data;
void Require(bool good, const char* message)
{
    if (!good) throw std::runtime_error(message);
}
void Reset()
{
    fake = flexiv::rdk::tilt_force_test::Data {};
    flexiv::rdk::ResetFakeScheduler();
    g_signal = 0; g_stop_sched = false; g_task_failed = false;
}
bool Fails(const char* expected)
{
    try { RunFloating("offline-only"); }
    catch (const std::exception& error) {
        Require(std::string(error.what()).find(expected) != std::string::npos, error.what());
        return true;
    }
    return false;
}
} // namespace

int main()
{
    try {
        Reset();
        const std::vector<double> damping {5, 5, 2.5, 2.5, 0.5, 0.5, 0.5};
        const std::vector<double> velocity {0.1, -0.1, 0.2, -0.2, 0.3, -0.3, 0.4};
        for (auto& [_, state] : fake.states) state.dtheta = velocity;
        fake.after_stream = [] { OnSignal(SIGINT); };
        size_t packets_at_stop = 0;
        fake.after_stop = [&] {
            Require(rdk::fake_scheduler_joined && rdk::fake_scheduler_destroyed,
                "Stop was called before joining the callback");
            packets_at_stop = fake.torque_packets.size();
        };
        Require(RunFloating("offline-only") == 128 + SIGINT, "interrupt status was lost");
        Require(fake.stops == 1 && fake.stopped && fake.mode == rdk::Mode::IDLE,
            "Ctrl+C failed to stop into IDLE hold");
        Require(fake.stop_reads > 1 && !fake.torque_packets.empty(), "stop was not confirmed");
        Require(fake.torque_packets.size() == packets_at_stop, "torque followed Stop");
        for (const auto& [_, cmd] : fake.torque_packets.front()) {
            Require(cmd.enable_gravity_comp && cmd.enable_soft_limits, "floating compensation changed");
            for (size_t i = 0; i < damping.size(); ++i)
                Require(std::abs(cmd.tau_d[i] + damping[i] * velocity[i]) < 1e-12,
                    "floating damping changed");
        }
        Reset(); fake.fail_stream = true;
        Require(Fails("callback failed") && fake.stops == 1, "callback failure skipped Stop");
        Reset(); fake.after_stream = [] { fake.fault = true; };
        Require(Fails("callback failed") && fake.stops == 1, "fault skipped Stop");
        Reset(); fake.fail_home = true;
        Require(Fails("Home failure") && fake.stops == 1 && fake.torque_packets.empty(),
            "Home failure advanced to torque control");
        Reset(); rdk::fake_scheduler_throw_on_start = true;
        Require(Fails("scheduler Start failure") && fake.stops == 1,
            "scheduler startup failure skipped Stop");
        Reset(); fake.after_stream = [] { OnSignal(SIGTERM); };
        rdk::fake_scheduler_throw_on_stop = true;
        Require(Fails("scheduler Stop failure") && fake.stops == 1 && rdk::fake_scheduler_destroyed,
            "scheduler cleanup failure skipped robot Stop");
        Reset(); fake.after_stream = [] { OnSignal(SIGHUP); }; fake.fail_stop = true;
        Require(Fails("Stop failure"), "Stop failure was reported as success");
        Reset(); fake.states.at(rdk::JointGroup::ARM_1).dtheta.clear();
        Require(Fails("callback failed") && fake.stops == 1 && fake.torque_packets.empty(),
            "invalid velocity dimensions reached torque streaming");
        Reset(); fake.states.at(rdk::JointGroup::ARM_1).dtheta[0] = std::nan("");
        Require(Fails("callback failed") && fake.stops == 1 && fake.torque_packets.empty(),
            "non-finite velocity reached torque streaming");
        Reset(); OnSignal(SIGINT);
        Require(Fails("interrupted") && fake.controls == 0, "startup interruption wrote commands");
        std::cout << "PASS floating damping, signal/fault cleanup, joined callbacks and IDLE hold confirmation\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "TEST_FAILED: " << error.what() << '\n';
        return 1;
    }
}
