// Exercises the task against in-memory Robot and ordinary-thread Scheduler only.
#define main TiltForceMain
#include "../third_party/flexiv_rdk/example/arm1_tcp_tilt_force.cpp"
#undef main

namespace {
namespace rdk = flexiv::rdk;
using namespace tilt_force;
auto& fake = rdk::tilt_force_test::data;
void Require(bool good, const char* message) { if (!good) throw std::runtime_error(message); }
void Reset()
{
    fake = rdk::tilt_force_test::Data {};
    rdk::ResetFakeScheduler(); interrupted.store(0);
}
Options Fast()
{
    Options opt;
    opt.serial = "offline-only"; opt.angle = 0.01; opt.tilt_duration = 0.005;
    opt.force_duration = 0.01; opt.finish_timeout = 0.05;
    opt.startup_timeout = 0.1; opt.home_timeout = 0.1;
    opt.stop_timeout = 0.4;
    return opt;
}
bool Fails(const Options& opt, const char* message)
{
    try { Run(opt); }
    catch (const std::exception& error) {
        Require(std::string(error.what()).find(message) != std::string::npos, error.what()); return true;
    }
    return false;
}
Options Arguments(std::vector<std::string> flags)
{
    flags.insert(flags.begin(), {"test", "offline-only"});
    std::vector<char*> args;
    for (auto& value : flags) args.push_back(value.data());
    return Parse(static_cast<int>(args.size()), args.data());
}
void PairedHold(const Pose& hold)
{
    Require(fake.primitive_commands.size() == 1 && fake.homes == 1, "Home was repeated or absent");
    const auto& primitives = fake.primitive_commands.front();
    Require(primitives.size() == 2 && primitives.at(kArm1).pt_name == "Home"
        && primitives.at(kArm2).pt_name == "Hold", "Home did not include ARM_2 Hold");
    for (const auto& [_, command] : primitives) {
        Require(!command.external_axis_control && command.sync_motion_mode == rdk::SyncMotionMode::DISABLE,
            "Home / Hold enabled external axes or synchronized motion");
    }
    Require(!fake.packets.empty(), "no RT packets");
    for (const auto& packet : fake.packets) {
        Require(packet.size() == 2 && packet.contains(kArm1) && packet.contains(kArm2),
            "ARM_1 RT command did not include ARM_2 hold");
        Require(packet.at(kArm2).pose_d == hold, "ARM_2 hold was recaptured or changed");
        for (double value : packet.at(kArm2).wrench_d) Require(value == 0, "ARM_2 received force");
    }
}
} // namespace

int main()
{
    try {
        const Eigen::Quaterniond q0(Eigen::AngleAxisd(0.7, Eigen::Vector3d::UnitY()));
        const Pose start {0.4, 0.2, 0.12, q0.w(), q0.x(), q0.y(), q0.z()};
        for (int axis = 0; axis < 3; ++axis) for (double angle : {-15., 15.}) {
            auto opt = Fast(); opt.axis = axis; opt.angle = angle;
            const TiltPlan plan(start, opt);
            const Eigen::Quaterniond expected = q0 * Eigen::AngleAxisd(angle * kPi / 180,
                Eigen::Vector3d::Unit(axis));
            Require(Rotation(plan.target).angularDistance(expected) < 1e-12, "rotation was not local TCP");
            for (size_t i = 0; i < 3; ++i) Require(plan.target[i] == start[i], "tilt translated TCP");
            Require(Rotation(plan.At(0)).angularDistance(q0) < 1e-12, "first target jumped");
        }
        Require(Arguments({}).angle == 15 && Arguments({"--x", "-15"}).angle == -15, "bad angle parsing");
        for (const auto& flags : std::vector<std::vector<std::string>> {
                 {"--x", "--15"}, {"--x", "nan"}, {"--x", "15garbage"}, {"--x", "181"},
                 {"--x", "10", "--y", "15"}, {"--arm", "ARM_2"}, {"--home-timeout", "0"},
                 {"--force", "inf"}, {"--force-duration", "-1"}, {"--x"},
                 {"--stop-timeout", "0.2"}, {"--stop-timeout", "nan"}}) {
            bool failed = false;
            try { Arguments(flags); } catch (...) { failed = true; }
            Require(failed, "invalid argument accepted");
        }
        Reset(); auto opt = Fast(); const auto hold = fake.states.at(kArm2).tcp_pose;
        Require(Run(opt) == 0, "full sequence failed"); PairedHold(hold);
        Require(fake.force_enables == 1 && fake.stops == 1, "wrong force / Stop count");
        Require(fake.mode == rdk::Mode::IDLE && fake.stopped && fake.stop_reads > 1,
            "completed without confirming stopped IDLE");
        Require(fake.axes.at(kArm1) == kForceZ && fake.axes.at(kArm2) == kMotionOnly, "wrong force axes");
        Require(fake.frames.at(kArm1) == rdk::CoordType::TCP, "force frame is not TCP");
        Require(fake.packets.back().at(kArm1).wrench_d[2] == -5, "wrong force sign");
        Require(fake.reference_q.at(kArm1) == fake.home_q, "used pre-Home ARM_1 joints");
        Reset(); opt = Fast(); opt.preview = true; fake.ready = false;
        rdk::fake_scheduler_throw_in_constructor = true;
        Require(Run(opt) == 0 && fake.controls == 0 && fake.homes == 0, "preview wrote commands");
        Reset(); opt = Fast(); opt.tilt_only = true;
        const Eigen::Quaterniond home_rotation(Eigen::AngleAxisd(0.7, Eigen::Vector3d::UnitZ()));
        fake.home_pose = {0.41, 0.21, 0.13, home_rotation.w(), home_rotation.x(), home_rotation.y(), home_rotation.z()};
        Require(Run(opt) == 0 && fake.force_enables == 0, "tilt-only enabled force"); PairedHold(hold);
        Require(Rotation(fake.packets.front().at(kArm1).pose_d).angularDistance(home_rotation) < 1e-12,
            "tilt origin did not use post-Home TCP");
        // Reproduce the reported ARM_2 drift without turning it into an abort.
        // Neither old TCP boxes, wrench thresholds, force tracking nor stale-state
        // guards are part of this task; every packet must still contain Hold.
        Reset(); opt = Fast(); fake.stamps = false; fake.contact = false;
        fake.home_pose[0] = 0.6; fake.home_pose[2] = 0.3;
        fake.after_stream = [] {
            fake.states.at(kArm2).q[0] += 0.002;
            fake.states.at(kArm2).tcp_pose[0] += 0.01;
            fake.states.at(kArm1).tcp_wrench[0] = 30;
            if (fake.force_enables) fake.states.at(kArm1).tcp_pose[2] += 0.05;
        };
        Require(Run(opt) == 0 && fake.force_enables == 1, "removed application guard still aborted");
        PairedHold(hold);
        for (int scenario = 0; scenario < 3; ++scenario) {
            Reset(); opt = Fast();
            if (scenario == 0) fake.fail_home = true;
            if (scenario == 1) fake.home_never_reaches = true;
            if (scenario == 2) fake.home_terminated = true;
            Require(Fails(opt, "Home") && fake.stops == 1 && fake.packets.empty(), "Home failure advanced to RT");
        }
        Reset(); opt = Fast(); fake.after_stream = [] { fake.states.at(kArm1).tcp_pose[0] += 0.01; };
        Require(Fails(opt, "Tilt arrival timeout") && fake.force_enables == 0 && fake.stops == 1,
            "force started before tilt arrival");
        Reset(); opt = Fast(); fake.fail_stream = true;
        Require(Fails(opt, "injected stream failure") && fake.stops == 1, "SDK failure was not propagated");
        Reset(); opt = Fast(); fake.after_stream = [] { fake.fault = true; };
        Require(Fails(opt, "Robot fault") && fake.stops == 1, "SDK fault was ignored");
        Reset(); opt = Fast(); fake.after_stream = [] { interrupted.store(SIGINT); };
        Require(Fails(opt, "Interrupted") && fake.stops == 1, "interrupt did not Stop");
        Reset(); opt = Fast(); rdk::fake_scheduler_skip_callbacks.store(true);
        Require(Fails(opt, "Scheduler readiness") && fake.controls == 0, "scheduler timeout issued Home");
        Reset(); opt = Fast(); opt.tilt_only = true; fake.fail_stop = true;
        Require(Fails(opt, "Stop failure"), "Stop failure was reported as success");
        for (int scenario = 0; scenario < 4; ++scenario) {
            Reset(); opt = Fast(); opt.tilt_only = true;
            if (scenario == 0) fake.stop_confirms = false;
            if (scenario == 1) fake.stop_enters_idle = false;
            if (scenario == 2) fake.after_stop = [] { fake.connected = false; };
            if (scenario == 3) fake.after_stop = [] { fake.fault = true; };
            Require(Fails(opt, scenario < 2 ? "Stop confirmation timeout"
                : scenario == 2 ? "Disconnected" : "Robot fault"),
                "unconfirmed stop was reported as success");
        }
        Reset(); opt = Fast(); opt.stop_timeout = 0.8;
        Clock::time_point stop_begin;
        size_t packets_at_stop = 0;
        fake.after_stop = [&] {
            Require(rdk::fake_scheduler_joined && rdk::fake_scheduler_destroyed,
                "Stop raced an active callback");
            stop_begin = Clock::now(); packets_at_stop = fake.packets.size();
        };
        fake.after_stop_check = [&] {
            // Simulate a brief return of motion after the first stopped report.
            const double elapsed = Seconds(stop_begin);
            fake.stopped = !(elapsed >= 0.08 && elapsed < 0.12);
        };
        Require(Run(opt) == 0 && Seconds(stop_begin) >= 0.32,
            "transient stopped report was accepted without settling");
        Require(fake.packets.size() == packets_at_stop, "commands followed Stop");
        Reset(); opt = Fast();
        rdk::Robot robot("offline-only");
        TiltPlan expired_plan(fake.states.at(kArm1).tcp_pose, opt);
        Task expired(robot, opt, expired_plan, hold);
        expired.force_started = true; expired.force_begin = Clock::now() - std::chrono::seconds(1);
        expired.stage.store(Stage::Force);
        expired.Tick();
        Require(expired.stage.load() == Stage::Done && fake.packets.empty(),
            "expired force phase sent another force packet");
        std::cout << "PASS paired Home/Hold and RT hold, local tilt/force, removed guards and SDK error handling\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "TEST_FAILED: " << error.what() << '\n'; return 1;
    }
}
