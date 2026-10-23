// Offline only: fake Robot, vendor value types, no SDK runtime.
#define main Arm1FloatingMain
#include "../third_party/flexiv_rdk/example/arm1_joint_floating_arm2_hold.cpp"
#undef main

namespace {
using namespace arm1_floating;
auto& fake = rdk::floating_test::data;
void Require(bool good, const char* message) { if (!good) throw std::runtime_error(message); }
void Reset() { fake = rdk::floating_test::Data {}; interrupted = 0; }
Options Fast()
{
    Options opt; opt.serial = "offline-only"; opt.duration = 0.025;
    opt.home_timeout = 0.06; opt.stop_timeout = 0.3; return opt;
}
bool Fails(const Options& opt, const char* expected)
{
    try { Run(opt); }
    catch (const std::exception& error) {
        Require(std::string(error.what()).find(expected) != std::string::npos, error.what());
        return true;
    }
    return false;
}
Options Arguments(std::vector<std::string> values)
{
    values.insert(values.begin(), {"test", "offline-only"});
    std::vector<char*> argv; for (auto& value : values) argv.push_back(value.data());
    return Parse(static_cast<int>(argv.size()), argv.data());
}
} // namespace

int main()
{
    try {
        for (const auto& flags : std::vector<std::vector<std::string>> {
                 {"--arm", "ARM_2"}, {"--unknown"}, {"--damping-level", "1"},
                 {"--max-velocity", "0.01"}, {"--duration", "nan"}, {"--stop-timeout", "0.2"},
                 {"--damping-level", "0", "0", "0", "0", "0", "0", "101"}}) {
            bool failed = false;
            try { Arguments(flags); } catch (...) { failed = true; }
            Require(failed, "invalid arguments accepted");
        }
        Require(Arguments({}).damping_level == std::vector<double>(7, 0), "wrong default damping levels");
        Reset(); auto opt = Fast();
        Require(Run(opt) == 0 && fake.stops == 1 && fake.stopped && fake.mode == rdk::Mode::IDLE,
            "full sequence did not finish stopped in IDLE");
        Require(fake.commands.size() == 2 && fake.commands[0].at(kArm1).pt_name == "Home"
            && fake.commands[1].at(kArm1).pt_name == "FloatingJoint", "wrong ARM_1 sequence");
        for (const auto& pair : fake.commands) {
            Require(pair.size() == 2 && pair.at(kArm2).pt_name == "Hold", "ARM_2 did not receive Hold");
            for (const auto& [_, command] : pair)
                Require(!command.external_axis_control && command.sync_motion_mode == rdk::SyncMotionMode::DISABLE,
                    "external or synchronized motion enabled");
        }
        const auto& inputs = fake.commands[1].at(kArm1).input_params;
        Require(std::get<std::vector<double>>(inputs.at("dampingLevel")) == std::vector<double>(7, 0),
            "RT torque gains were substituted for primitive levels");
        Require(std::get<std::vector<double>>(inputs.at("floatingJoint")) == std::vector<double>(7, 1),
            "wrong floating joint selection");
        Require(std::get<double>(inputs.at("maxVel")) == 0.2, "wrong native velocity setting");
        Require(Arguments({"--skip-home"}).skip_home, "skip-home option was ignored");
        Reset(); opt = Fast(); opt.skip_home = true; fake.require_home = false;
        Require(Run(opt) == 0 && fake.homes == 0 && fake.floats == 1 && fake.stops == 1,
            "current-position floating unexpectedly executed Home");
        Require(fake.commands.size() == 1 && fake.commands[0].at(kArm1).pt_name == "FloatingJoint"
            && fake.commands[0].at(kArm2).pt_name == "Hold",
            "current-position floating did not pair ARM_1 FloatingJoint with ARM_2 Hold");
        Reset(); opt = Fast(); opt.preview = true;
        Require(Run(opt) == 0 && fake.controls == 0, "preview issued control");
        Reset(); opt = Fast(); fake.info.single_arm_groups.erase(kArm2);
        Require(Fails(opt, "both required") && fake.controls == 0, "missing ARM_2 was accepted");
        for (const auto status : {rdk::OperationalStatus::IN_AUTO_MODE, rdk::OperationalStatus::IN_MANUAL_MODE}) {
            Reset(); opt = Fast(); fake.ready = false; fake.status = status; opt.auto_servo_on = true;
            Require(Fails(opt, "AUTO (Remote)") && fake.controls == 0, "non-Remote mode wrote controls");
        }
        Reset(); opt = Fast(); fake.ready = false; fake.status = rdk::OperationalStatus::NOT_SERVO_ON;
        Require(Fails(opt, "AUTO (Remote)") && fake.servos == 0, "unexpected automatic ServoOn");
        Reset(); opt = Fast(); fake.ready = false; fake.status = rdk::OperationalStatus::NOT_SERVO_ON;
        opt.auto_servo_on = true;
        Require(Run(opt) == 0 && fake.servos == 1, "authorized ServoOn failed");
        for (int scenario = 0; scenario < 4; ++scenario) {
            Reset(); opt = Fast();
            if (scenario == 0) fake.fail_home = true;
            if (scenario == 1) fake.home_never_reaches = true;
            if (scenario == 2) fake.home_terminated = true;
            if (scenario == 3) fake.hold_terminated = true;
            Require(Fails(opt, scenario == 3 ? "Hold terminated" : "Home")
                && fake.floats == 0 && fake.stops == 1, "Home/Hold failure advanced to floating");
        }
        Reset(); opt = Fast(); fake.on_floating_read = [] { fake.hold_terminated = true; };
        Require(Fails(opt, "Hold terminated") && fake.stops == 1, "floating ignored lost ARM_2 Hold");
        Reset(); opt = Fast(); fake.floating_terminated = true;
        Require(Fails(opt, "before activation") && fake.stops == 1, "inactive floating reported success");
        Reset(); opt = Fast(); fake.on_floating_read = [] { Interrupt(SIGINT); };
        Require(Fails(opt, "Interrupted") && fake.stops == 1, "interrupt did not stop");
        Reset(); opt = Fast(); fake.on_floating_read = [] { fake.fault = true; };
        Require(Fails(opt, "faulted") && fake.stops == 1, "fault did not stop");
        Reset(); opt = Fast(); fake.stop_confirms = false;
        Require(Fails(opt, "Stop confirmation timeout"), "moving robot reported stopped");
        Reset(); opt = Fast(); fake.fail_stop = true;
        Require(Fails(opt, "Stop failure"), "failed Stop reported success");
        std::cout << "PASS ARM_1 Home/FloatingJoint, paired ARM_2 Hold, native parameters and stop paths\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "TEST_FAILED: " << error.what() << '\n'; return 1;
    }
}
