// 工具负载标定：用 RDK 2.1 官方的 Tool::CalibratePayloadParams 标定法兰上工具的质量、质心、惯量，
// 和 STL 算出的参考值对比，确认后写成工具并在该臂上启用。
// Tool payload calibration: calibrate the mass, CoM and inertia of the tool on the flange with RDK 2.1's official
// Tool::CalibratePayloadParams, compare with reference values computed from the STL, and, once confirmed, save them as
// a tool and activate it on the arm.
//
// 流程照 tool.hpp 里 CalibratePayloadParams 的官方步骤 1–6 / Sequence follows the official steps 1–6 in tool.hpp:
//   ① 连接、检查、上伺服，切到 IDLE（标定只能在 IDLE 下调用）；打印工具列表和两条臂当前的工具。
//      Connect, check, servo on, switch to IDLE (calibration is IDLE-only); print the tool list and both arms' tools.
//   ② --arm 指定的臂切到 Flange，不启用任何工具（官方步骤 1）。
//      Switch the --arm arm to Flange, i.e. no active tool (official step 1).
//   ③ 装着工具标定，tool_mounted = true（官方步骤 3）。按 Enter 后机器人自己动。
//      Calibrate with the tool mounted, tool_mounted = true (official step 3). The robot moves by itself after Enter.
//   ④ 本机没有法兰 6 轴力传感器（末端力由关节力矩估算），官方建议再做一步：拆下工具后按 Enter，
//      再标定一次，tool_mounted = false（官方步骤 4–5）。--mounted-only 跳过。
//      This robot has no flange 6-axis F/T sensor (the TCP wrench is estimated from joint torques), so the docs
//      recommend one more step: unmount the tool, press Enter, calibrate again with tool_mounted = false (official
//      steps 4–5). --mounted-only skips it.
//   ⑤ 打印结果和 STL 参考值；输入 y 才把最后一次的结果写成工具 --tool-name 并在该臂启用（官方步骤 6），
//      否则恢复原来的工具。不管结果如何都写一份 JSON 记录。
//      Print the results next to the STL reference; only on "y" save the last result as tool --tool-name and activate
//      it on the arm (official step 6), otherwise restore the previous tool. A JSON record is written in every case.
//
// 双臂：CalibratePayloadParams 没有关节组参数，RDK 2.1 头文件和官方 API 文档也没说双臂机器人标定的是哪条臂。
// 所以标定期间另开线程每 10 ms 采样两条臂的关节位置：只有 --arm 那条臂动了、另一条没动，才允许写入。
// Dual arm: CalibratePayloadParams takes no joint group, and neither the RDK 2.1 header nor the official API docs say
// which arm a dual-arm robot calibrates. So a second thread samples both arms' joint positions every 10 ms during
// calibration: the result may be saved only if the --arm arm moved and the other did not.
//
// --no-calibration：跳过 ②–④，直接写 --weighed-mass + STL 质心 + STL 形状 × 称重质量，机器人不动。
// 很轻的工具用它：本机末端力由关节力矩估算，静止噪声和几十克工具的重力同一个量级，标不出质心。
// --no-calibration: skips ②–④ and writes --weighed-mass + STL CoM + STL shape × weighed mass; the robot does not move.
// Meant for very light tools: this robot estimates the TCP wrench from joint torques, and its resting noise is of the
// same order as the weight of a tool of a few tens of grams, so the CoM cannot be calibrated.
//
// 不用实时调度器，不用 sudo。/ No RT scheduler, no sudo.

#include <flexiv/rdk/robot.hpp>
#include <flexiv/rdk/tool.hpp>
#include <flexiv/rdk/utility.hpp>

#include <Eigen/Eigenvalues>  // InertiaPhysical 的主惯量；RDK 本身依赖 Eigen 3.4。/ Principal moments; RDK already needs Eigen 3.4.
#include <poll.h>  // ReadLine：等终端输入时还能响应 Ctrl+C。/ ReadLine: stay responsive to Ctrl+C while waiting for input.
#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cctype>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace rdk = flexiv::rdk;

// ================================ 参数：在这里改 / Parameters: edit here ================================
const std::string kToolName = "suyi_Nov12";  // 写入的工具名，--tool-name= 覆盖。/ Tool name to save; --tool-name= overrides.
// STL 参考值，来自 suyi_flexiv_elements_studio_inputs.json（"suyi Nov12.stl"，假定实心均匀、STL 坐标系 = 法兰坐标系）。
// 只用来对比，不写入（--stl-inertia 时惯量除外）。
// STL reference from suyi_flexiv_elements_studio_inputs.json ("suyi Nov12.stl", assuming a uniform solid and STL frame
// = flange frame). For comparison only, never written (except the inertia with --stl-inertia).
constexpr std::array<double, 3> kStlCoM = {-0.07602872320952206e-3, 6.611114542073878e-13, 34.96305633028026e-3};  // [m]
// 该文件给 Elements 填的惯量 ÷ 质量 [m²]，顺序 Ixx Iyy Izz Ixy Ixz Iyz（同 ToolParams::inertia）；乘以质量 [kg] 得 [kg·m²]。
// That file's Elements inertia divided by mass [m²], ordered Ixx Iyy Izz Ixy Ixz Iyz (as ToolParams::inertia); times
// mass [kg] gives [kg·m²].
constexpr std::array<double, 6> kStlInertiaPerKg
    = {0.0014213721569165483, 0.0014194656610675965, 0.0004364066890933909, 0.0, -2.2780529175837764e-06, 0.0};
// 判定"这条臂在标定时动过"：7 个关节里离开起点最远的那个超过它 [rad]。静止保持时的抖动远小于它。
// "This arm moved during calibration": the joint farthest from its start exceeds this [rad]. Holding jitter is far below it.
constexpr double kMovedThreshold = 0.02;
constexpr double kLightPayload = 0.2;  // Elements 手册：负载 ≤ 200 g 时自动标定可能不准 [kg]。/ Elements manual: auto calibration may be inaccurate at ≤ 200 g [kg].
constexpr double kOperationalTimeout = 30.0;  // 上伺服后等待就绪的最长时间 [s]。/ Longest wait for operational after servo-on [s].
constexpr auto kSamplePeriod = std::chrono::milliseconds(10);  // 标定期间采样两条臂的周期。/ Arm sampling period during calibration.
constexpr auto kProgressPeriod = std::chrono::seconds(5);  // 标定期间打印进度的周期。/ Progress print period during calibration.
// ====================================================================================================

constexpr std::array<rdk::JointGroup, 2> kArms = {rdk::JointGroup::ARM_1, rdk::JointGroup::ARM_2};
const std::array<std::string, 6> kInertiaNames = {"Ixx", "Iyy", "Izz", "Ixy", "Ixz", "Iyz"};

struct Params
{
    std::string robot_sn;
    size_t arm = 0;  // 工具装在哪条臂：kArms 的下标。/ Arm carrying the tool, as an index into kArms.
    std::string tool_name = kToolName;
    double weighed_mass = 0.0;  // 秤的读数 [kg]，只用来对比；0 = 没给。/ Scale reading [kg], comparison only; 0 = not given.
    bool has_tcp = false;
    std::array<double, 3> tcp = {};  // --tcp-mm 给的 TCP 位置 [m]，姿态与法兰相同。/ TCP position from --tcp-mm [m], flange orientation.
    bool stl_inertia = false;
    bool mounted_only = false;
    bool no_calibration = false;
    std::string log_path;  // 空 = logs/tool_payload_<日期时间>.json。/ Empty = logs/tool_payload_<date_time>.json.
    bool clear_fault = false;
};

// 一次 CalibratePayloadParams 的结果和这期间两条臂的运动。/ One CalibratePayloadParams result and both arms' motion meanwhile.
struct Calibration
{
    bool tool_mounted = true;
    double seconds = 0.0;
    rdk::ToolParams result;
    std::array<double, 2> max_displacement = {};  // ARM_1、ARM_2 离开起点的最大关节位移 [rad]。/ Max joint displacement from start [rad].
    uint64_t samples = 0;
};

// 写进 JSON 记录的全部内容。/ Everything that goes into the JSON record.
struct Record
{
    std::string time, robot_sn, software_ver, arm, tool_name;
    std::string source;  // 写入值的来源："calibration" 或 "weighed_mass+stl"。/ Source of the values: "calibration" or "weighed_mass+stl".
    double weighed_mass = 0.0;
    std::string previous_tool;
    bool has_previous_params = false;
    rdk::ToolParams previous_params;
    std::vector<Calibration> calibrations;
    bool applied = false;
    rdk::ToolParams applied_params;
    std::string not_applied_reason, active_tool_after, error;
};

// 第一次信号置 true：采样线程发 Stop()，主线程收尾。第二次信号直接退出，防止标定卡住时退不出来。
// First signal sets this: the sampler sends Stop() and main shuts down. A second signal exits at once, in case
// calibration hangs.
std::atomic<bool> g_stop = {false};

void OnSignal(int)
{
    if (g_stop.exchange(true)) {
        std::_Exit(130);
    }
}

void PrintHelp()
{
    std::cout << "用法：.build/script/calibrate_tool_payload <序列号> [选项]   （不用 sudo）\n"
                 "  <序列号>          完整序列号，不要有空格，例如 Enlight-LL-123456\n"
                 "  --arm=N           工具装在 ARM_N 上，N = 1 或 2，默认 1\n"
                 "  --tool-name=NAME  覆盖 kToolName：写入的工具名，只能用字母、数字、_、-，不能是 Flange。\n"
                 "                    已有同名工具就更新它，没有就新建\n"
                 "  --weighed-mass=KG 秤称的整个工具（含螺丝、夹具）质量 [kg]，只用来和标定结果对比，不写入\n"
                 "                    （--no-calibration 时除外）\n"
                 "  --no-calibration  不标定，机器人不动：直接写入 --weighed-mass + STL 质心 + STL 形状 × 称重质量。\n"
                 "                    要和 --weighed-mass 一起用\n"
                 "  --tcp-mm=X,Y,Z    写入的 TCP 位置 [mm]，姿态与法兰相同。不给时：更新已有工具保持它原来的 TCP，\n"
                 "                    新建工具 TCP 在法兰中心\n"
                 "  --stl-inertia     写入的惯量改用 STL 形状 × 标定质量（标定出的惯量不合物理时用）\n"
                 "  --mounted-only    只做装着工具的那次标定，跳过拆下工具后的第二次\n"
                 "  --log=FILE.json   结果记录；默认 logs/tool_payload_<日期时间>.json。文件已经存在时不运行\n"
                 "  --clear-fault     启动时若有故障先尝试清除；不加则直接退出\n"
                 "  -h, --help        显示本说明\n";
}

bool ParseNumber(const std::string& text, double& value)  // 整个字符串是一个有限的数才算成功。/ Succeeds only if the whole string is one finite number.
{
    char* end = nullptr;
    value = std::strtod(text.c_str(), &end);
    return end != text.c_str() && *end == '\0' && std::isfinite(value);
}

// arg 是 "name=值" 时取出值。/ Extracts the value when arg is "name=value".
bool OptionValue(const std::string& arg, const std::string& name, std::string& value)
{
    if (!arg.starts_with(name + "=")) {
        return false;
    }
    value = arg.substr(name.size() + 1);
    return true;
}

// 读取命令行；未给出的用上方常量。出错返回原因。/ Reads the command line; unset options use the constants above. Returns the error, if any.
std::string ParseArgs(int argc, char* argv[], Params& params)
{
    params.robot_sn = argv[1];
    if (params.robot_sn.starts_with("-")) {
        return "第一个参数必须是序列号";
    }
    for (int i = 2; i < argc; ++i) {
        const std::string arg = argv[i];
        std::string value;
        if (OptionValue(arg, "--arm", value)) {
            if (value != "1" && value != "2") {
                return "--arm= 后面要跟 1 或 2";
            }
            params.arm = value == "1" ? 0 : 1;
        } else if (OptionValue(arg, "--tool-name", value)) {
            params.tool_name = value;
        } else if (OptionValue(arg, "--weighed-mass", value)) {
            if (!ParseNumber(value, params.weighed_mass) || !(params.weighed_mass > 0.0)) {
                return "--weighed-mass= 后面要跟 1 个大于 0 的数 [kg]";
            }
        } else if (OptionValue(arg, "--tcp-mm", value)) {
            std::istringstream iss(value);
            std::string item;
            size_t count = 0;
            while (std::getline(iss, item, ',')) {
                double mm = 0.0;
                if (count >= 3 || !ParseNumber(item, mm)) {
                    return "--tcp-mm= 后面要跟逗号分开的 3 个数 [mm]";
                }
                params.tcp[count++] = mm * 1e-3;
            }
            if (count != 3 || value.ends_with(",")) {
                return "--tcp-mm= 后面要跟逗号分开的 3 个数 [mm]";
            }
            params.has_tcp = true;
        } else if (OptionValue(arg, "--log", value)) {
            if (value.empty()) {
                return "--log= 后面要跟文件名";
            }
            params.log_path = value;
        } else if (arg == "--stl-inertia") {
            params.stl_inertia = true;
        } else if (arg == "--mounted-only") {
            params.mounted_only = true;
        } else if (arg == "--no-calibration") {
            params.no_calibration = true;
        } else if (arg == "--clear-fault") {
            params.clear_fault = true;
        } else {
            return "不认识的参数 " + arg;
        }
    }
    if (params.no_calibration && !(params.weighed_mass > 0.0)) {
        return "--no-calibration 要和 --weighed-mass= 一起用";
    }
    if (params.no_calibration && (params.mounted_only || params.stl_inertia)) {
        return "--no-calibration 不标定，不能和 --mounted-only、--stl-inertia 一起用";
    }
    // 上方常量也一样要过这一关。/ The constant above must pass too.
    const bool name_ok = !params.tool_name.empty()
                         && std::all_of(params.tool_name.begin(), params.tool_name.end(), [](unsigned char c) {
                                return std::isalnum(c) || c == '_' || c == '-';
                            });
    if (!name_ok || params.tool_name == "Flange") {
        return "工具名 \"" + params.tool_name + "\" 不行：只能用字母、数字、_、-，不能是 Flange";
    }
    return "";
}

std::string Fixed(double value, int digits)  // 定点小数，例如 Fixed(0.1234, 2) = "0.12"。/ Fixed-point text.
{
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(digits) << value;
    return oss.str();
}

std::string Sci(double value)  // 惯量这种很小的数用科学计数法。/ Scientific notation for tiny values such as inertia.
{
    std::ostringstream oss;
    oss << std::scientific << std::setprecision(3) << value;
    return oss.str();
}

// 本地时间，format 同 strftime。/ Local time, formatted as strftime.
std::string LocalTime(const char* format)
{
    const std::time_t now = std::time(nullptr);
    std::tm local = {};
    localtime_r(&now, &local);
    char text[64] = {};
    std::strftime(text, sizeof(text), format, &local);
    return text;
}

// 等用户输入一行；Ctrl+C 或输入结束（EOF）时返回 false。用 poll 轮询，Ctrl+C 不会卡在读终端上。
// Waits for one line of input; false on Ctrl+C or end of input (EOF). Polls so Ctrl+C never blocks on the terminal read.
bool ReadLine(std::string& line)
{
    while (!g_stop) {
        pollfd fd = {STDIN_FILENO, POLLIN, 0};
        const int ready = poll(&fd, 1, 200);
        if (ready > 0) {
            return static_cast<bool>(std::getline(std::cin, line)) && !g_stop;
        }
        if (ready < 0 && errno != EINTR) {
            return false;
        }
    }
    return false;
}

// 打印问题并等一行回答；被中断时抛出。/ Prints the question and waits for one line; throws if interrupted.
std::string Ask(const std::string& question)
{
    std::cout << question << std::flush;
    std::string line;
    if (!ReadLine(line)) {
        throw std::runtime_error("被中断，没有继续");
    }
    return line;
}

// 机器人没故障、已就绪；没上伺服就上伺服并等它就绪（E-stop 必须已松开）。
// Ensures no fault and operational; servos on and waits if needed (the E-stop must be released).
void EnsureOperational(rdk::Robot& robot)
{
    if (robot.fault()) {
        throw std::runtime_error("机器人有故障；确认原因后加 --clear-fault 重新运行");
    }
    if (robot.operational()) {
        return;
    }
    robot.ServoOn();
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(kOperationalTimeout);
    while (!robot.operational()) {
        if (g_stop) {
            throw std::runtime_error("等待机器人就绪时被中断");
        }
        if (std::chrono::steady_clock::now() > deadline) {
            throw std::runtime_error("等待机器人就绪超时（检查 E-stop、AUTO(Remote)）");
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
}

// 连接后的检查和上伺服；不满足就抛出，此时还没发过运动指令。
// Post-connection checks and servo-on; throws if anything is wrong, before any motion command has been sent.
void Prepare(rdk::Robot& robot, const Params& params)
{
    if (robot.fault()) {
        if (!params.clear_fault) {
            throw std::runtime_error("机器人有故障；确认原因后加 --clear-fault 重试");
        }
        if (!robot.ClearFault()) {
            throw std::runtime_error("故障无法清除");
        }
    }
    const auto info = robot.info();
    for (const auto group : kArms) {  // 采样线程要读两条臂。/ The sampler reads both arms.
        if (!info.single_arm_groups.contains(group)) {
            throw std::runtime_error("控制器没有单臂关节组 " + rdk::kJointGroupNames.at(group));
        }
    }
    if (!robot.all_stopped()) {  // 有别的程序在动机器人时不接管。/ Do not take over while another program moves the robot.
        throw std::runtime_error("机器人还在运动；先停止其他控制程序");
    }
    EnsureOperational(robot);
}

// 两组关节位置差的最大绝对值 [rad]。/ Largest absolute difference between two joint vectors [rad].
double MaxDistance(const std::vector<double>& q, const std::vector<double>& q0)
{
    double distance = 0.0;
    for (size_t j = 0; j < std::min(q.size(), q0.size()); ++j) {
        distance = std::max(distance, std::abs(q[j] - q0[j]));
    }
    return distance;
}

// 调一次 CalibratePayloadParams（阻塞到标定结束），同时另开线程采样两条臂离开起点多远，每 kProgressPeriod 打印一次。
// 第一次 Ctrl+C 时采样线程调 Stop()；它能不能停下控制器内部的标定没有文档，E-stop 才是一定能停住的办法。
// Calls CalibratePayloadParams once (blocks until calibration ends) while a second thread samples how far both arms
// have moved from their start, printing every kProgressPeriod. On the first Ctrl+C the sampler calls Stop(); whether
// that halts the controller's internal calibration is undocumented, the E-stop is the sure way to stop.
Calibration CalibrateWatched(rdk::Robot& robot, rdk::Tool& tool, bool tool_mounted)
{
    if (!robot.all_stopped()) {
        throw std::runtime_error("机器人还在运动，没有开始标定");
    }
    EnsureOperational(robot);
    Calibration cali;
    cali.tool_mounted = tool_mounted;
    const auto start_states = robot.states();
    const std::array<std::vector<double>, 2> q0
        = {start_states.at(kArms[0]).q, start_states.at(kArms[1]).q};
    const auto start = std::chrono::steady_clock::now();
    std::atomic<bool> done = {false};
    std::thread sampler([&] {
        bool stop_sent = false;
        auto next_print = start + kProgressPeriod;
        while (!done) {
            if (g_stop && !stop_sent) {
                stop_sent = true;
                std::cout << "收到 Ctrl+C，发 Stop() ……（机器人不停就拍 E-stop；再按一次 Ctrl+C 强制退出程序）"
                          << std::endl;
                try {
                    robot.Stop();
                } catch (const std::exception& e) {
                    std::cout << "Stop() 没有执行：" << e.what() << std::endl;
                }
            }
            try {
                const auto states = robot.states();
                for (size_t a = 0; a < kArms.size(); ++a) {
                    cali.max_displacement[a]
                        = std::max(cali.max_displacement[a], MaxDistance(states.at(kArms[a]).q, q0[a]));
                }
                ++cali.samples;
            } catch (const std::exception&) {
                // 偶尔读不到状态就跳过这一次。/ Skip a sample if states are briefly unavailable.
            }
            const auto now = std::chrono::steady_clock::now();
            if (now >= next_print) {
                next_print += kProgressPeriod;
                std::cout << "  标定中 " << Fixed(std::chrono::duration<double>(now - start).count(), 0)
                          << " s：离开起点最多 ARM_1 " << Fixed(cali.max_displacement[0], 3) << " rad，ARM_2 "
                          << Fixed(cali.max_displacement[1], 3) << " rad" << std::endl;
            }
            std::this_thread::sleep_for(kSamplePeriod);
        }
    });
    try {
        cali.result = tool.CalibratePayloadParams(tool_mounted);
    } catch (...) {
        done = true;
        sampler.join();
        throw;
    }
    done = true;
    sampler.join();
    cali.seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    if (g_stop) {
        throw std::runtime_error("标定期间被中断，结果不用");
    }
    std::cout << "标定完成，用时 " << Fixed(cali.seconds, 1) << " s；离开起点最多 ARM_1 "
              << Fixed(cali.max_displacement[0], 3) << " rad，ARM_2 " << Fixed(cali.max_displacement[1], 3)
              << " rad（采样 " << cali.samples << " 次）。" << std::endl;
    return cali;
}

// 只有 arm 那条臂动过、另一条没动过，才能确定标定的是 arm；否则返回原因。
// Only if the arm moved and the other did not is the calibration known to be of that arm; otherwise returns why.
std::string WrongArmReason(const Calibration& cali, size_t arm)
{
    const std::string step = cali.tool_mounted ? "装着工具的标定" : "拆下工具后的标定";
    const size_t other = 1 - arm;
    if (cali.max_displacement[arm] < kMovedThreshold) {
        return step + "期间 " + rdk::kJointGroupNames.at(kArms[arm]) + " 没动（最多 "
               + Fixed(cali.max_displacement[arm], 3) + " rad），标定的不是这条臂";
    }
    if (cali.max_displacement[other] >= kMovedThreshold) {
        return step + "期间 " + rdk::kJointGroupNames.at(kArms[other]) + " 也动了（最多 "
               + Fixed(cali.max_displacement[other], 3) + " rad），不能确定结果属于哪条臂";
    }
    return "";
}

// 惯量在物理上可能：主惯量都 > 0，且两个较小的之和不小于最大的。
// Physically possible inertia: all principal moments > 0 and the two smaller ones sum to at least the largest.
bool InertiaPhysical(const std::array<double, 6>& inertia)
{
    Eigen::Matrix3d matrix;
    matrix << inertia[0], inertia[3], inertia[4], inertia[3], inertia[1], inertia[5], inertia[4], inertia[5],
        inertia[2];
    const Eigen::Vector3d moments = Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d>(matrix).eigenvalues();  // 从小到大。/ Ascending.
    return moments[0] > 0.0 && moments[0] + moments[1] >= moments[2] * (1.0 - 1e-9);
}

std::array<double, 6> StlInertia(double mass)  // STL 形状 × 质量 [kg·m²]。/ STL shape times mass [kg·m²].
{
    std::array<double, 6> inertia;
    for (size_t i = 0; i < inertia.size(); ++i) {
        inertia[i] = kStlInertiaPerKg[i] * mass;
    }
    return inertia;
}

void PrintParams(const std::string& title, const rdk::ToolParams& p)
{
    std::cout << title << "：质量 " << Fixed(p.mass, 4) << " kg，质心 [" << Fixed(p.CoM[0] * 1e3, 2) << ", "
              << Fixed(p.CoM[1] * 1e3, 2) << ", " << Fixed(p.CoM[2] * 1e3, 2) << "] mm，惯量";
    for (size_t i = 0; i < p.inertia.size(); ++i) {
        std::cout << " " << kInertiaNames[i] << " " << Sci(p.inertia[i]);
    }
    std::cout << " kg·m²，TCP [" << Fixed(p.tcp_location[0] * 1e3, 1) << ", " << Fixed(p.tcp_location[1] * 1e3, 1)
              << ", " << Fixed(p.tcp_location[2] * 1e3, 1) << "] mm 四元数 [" << Fixed(p.tcp_location[3], 4) << ", "
              << Fixed(p.tcp_location[4], 4) << ", " << Fixed(p.tcp_location[5], 4) << ", "
              << Fixed(p.tcp_location[6], 4) << "]" << std::endl;
}

// 每次标定的结果和 STL 参考值并排打印；STL 一列的惯量 = STL 形状 × 最后一次标定的质量。
// Prints each calibration result next to the STL reference; the STL inertia column = STL shape × the last calibrated mass.
void PrintComparison(const std::vector<Calibration>& calis, double weighed_mass)
{
    const double mass = calis.back().result.mass;
    const auto stl_inertia = StlInertia(mass);
    std::cout << "\n⑤ 结果对比（mounted = ③ 装着工具，unmounted = ④ 拆下工具后；STL 一列：质量是 --weighed-mass，"
                 "惯量是 STL 形状 × 最后一次标定质量）\n";
    std::cout << std::left << std::setw(16) << "";
    for (const auto& cali : calis) {
        std::cout << std::setw(16) << (cali.tool_mounted ? "mounted" : "unmounted");
    }
    std::cout << "STL" << "\n";
    const auto row = [&](const std::string& name, auto value_of, const std::string& stl) {
        std::cout << std::setw(16) << name;
        for (const auto& cali : calis) {
            std::cout << std::setw(16) << value_of(cali.result);
        }
        std::cout << stl << "\n";
    };
    row("mass [kg]", [](const rdk::ToolParams& p) { return Fixed(p.mass, 4); },
        weighed_mass > 0.0 ? Fixed(weighed_mass, 4) : "-");
    for (size_t i = 0; i < 3; ++i) {
        row(std::string("CoM ") + "xyz"[i] + " [mm]", [i](const rdk::ToolParams& p) { return Fixed(p.CoM[i] * 1e3, 2); },
            Fixed(kStlCoM[i] * 1e3, 2));
    }
    for (size_t i = 0; i < 6; ++i) {
        row(kInertiaNames[i] + " [kg m^2]", [i](const rdk::ToolParams& p) { return Sci(p.inertia[i]); },
            Sci(stl_inertia[i]));
    }
    std::cout << std::right;
    const auto& last = calis.back().result;
    const double com_error = std::hypot(last.CoM[0] - kStlCoM[0], last.CoM[1] - kStlCoM[1], last.CoM[2] - kStlCoM[2]);
    std::cout << "最后一次标定的质心离 STL 质心 " << Fixed(com_error * 1e3, 1) << " mm。";
    if (weighed_mass > 0.0) {
        std::cout << "标定质量比称重" << (mass >= weighed_mass ? "多 " : "少 ")
                  << Fixed(std::abs(mass - weighed_mass) * 1e3, 0) << " g（"
                  << Fixed((mass - weighed_mass) / weighed_mass * 100.0, 0) << "%）。";
    }
    std::cout << std::endl;
    if ((weighed_mass > 0.0 ? weighed_mass : mass) <= kLightPayload) {
        std::cout << "注意：负载 ≤ " << kLightPayload * 1e3 << " g，Elements 手册提示自动负载标定可能不准。" << std::endl;
    }
}

std::string JsonString(const std::string& text)
{
    std::ostringstream oss;
    oss << '"';
    for (const unsigned char c : text) {
        if (c == '"' || c == '\\') {
            oss << '\\' << c;
        } else if (c < 0x20) {
            oss << "\\u" << std::hex << std::setw(4) << std::setfill('0') << static_cast<int>(c) << std::dec;
        } else {
            oss << c;
        }
    }
    oss << '"';
    return oss.str();
}

template <size_t N>
std::string JsonArray(const std::array<double, N>& values)
{
    std::ostringstream oss;
    oss << std::setprecision(12) << '[';
    for (size_t i = 0; i < N; ++i) {
        oss << (i ? ", " : "") << values[i];
    }
    oss << ']';
    return oss.str();
}

std::string JsonParams(const rdk::ToolParams& p)
{
    std::ostringstream oss;
    oss << std::setprecision(12) << "{\"mass_kg\": " << p.mass << ", \"CoM_m\": " << JsonArray(p.CoM)
        << ", \"inertia_kg_m2\": " << JsonArray(p.inertia) << ", \"tcp_location\": " << JsonArray(p.tcp_location)
        << "}";
    return oss.str();
}

void WriteJson(std::ostream& out, const Record& r)
{
    out << std::setprecision(12) << "{\n"
        << "  \"program\": \"calibrate_tool_payload\",\n"
        << "  \"time\": " << JsonString(r.time) << ",\n"
        << "  \"robot_sn\": " << JsonString(r.robot_sn) << ",\n"
        << "  \"software_ver\": " << JsonString(r.software_ver) << ",\n"
        << "  \"arm\": " << JsonString(r.arm) << ",\n"
        << "  \"tool_name\": " << JsonString(r.tool_name) << ",\n"
        << "  \"source\": " << JsonString(r.source) << ",\n"
        << "  \"weighed_mass_kg\": ";
    if (r.weighed_mass > 0.0) {
        out << r.weighed_mass;
    } else {
        out << "null";
    }
    out << ",\n  \"stl_reference\": {\"source\": \"suyi_flexiv_elements_studio_inputs.json\", \"CoM_m\": "
        << JsonArray(kStlCoM) << ", \"inertia_per_kg_m2\": " << JsonArray(kStlInertiaPerKg) << "},\n"
        << "  \"previous_tool\": " << JsonString(r.previous_tool) << ",\n"
        << "  \"previous_tool_params\": " << (r.has_previous_params ? JsonParams(r.previous_params) : "null")
        << ",\n  \"calibrations\": [";
    for (size_t i = 0; i < r.calibrations.size(); ++i) {
        const auto& c = r.calibrations[i];
        out << (i ? "," : "") << "\n    {\"tool_mounted\": " << (c.tool_mounted ? "true" : "false")
            << ", \"seconds\": " << c.seconds << ", \"samples\": " << c.samples
            << ", \"max_joint_displacement_rad\": {\"ARM_1\": " << c.max_displacement[0]
            << ", \"ARM_2\": " << c.max_displacement[1] << "}, \"result\": " << JsonParams(c.result) << "}";
    }
    out << (r.calibrations.empty() ? "" : "\n  ") << "],\n"
        << "  \"applied\": " << (r.applied ? "true" : "false") << ",\n"
        << "  \"applied_params\": " << (r.applied ? JsonParams(r.applied_params) : "null") << ",\n"
        << "  \"not_applied_reason\": " << JsonString(r.not_applied_reason) << ",\n"
        << "  \"active_tool_after\": " << JsonString(r.active_tool_after) << ",\n"
        << "  \"error\": " << JsonString(r.error) << "\n"
        << "}\n";
}

int main(int argc, char* argv[])
{
    if (argc < 2 || rdk::utility::ProgramArgsExistAny(argc, argv, {"-h", "--help"})) {
        PrintHelp();
        return argc < 2 ? 2 : 0;
    }
    Params params;
    if (const auto error = ParseArgs(argc, argv, params); !error.empty()) {
        std::cerr << "参数错误：" << error << "\n\n";
        PrintHelp();
        return 2;
    }
    Record record;
    record.time = LocalTime("%Y-%m-%dT%H:%M:%S");
    record.robot_sn = params.robot_sn;
    record.arm = rdk::kJointGroupNames.at(kArms[params.arm]);
    record.tool_name = params.tool_name;
    record.source = params.no_calibration ? "weighed_mass+stl" : "calibration";
    record.weighed_mass = params.weighed_mass;
    if (params.log_path.empty()) {
        params.log_path = "logs/tool_payload_" + LocalTime("%Y%m%d_%H%M%S") + ".json";
    }
    // 连机器人之前打开：路径不对时机器人还没动就退出。/ Opened before connecting, so a bad path exits before the robot moves.
    std::error_code exists_error;
    if (std::filesystem::exists(params.log_path, exists_error)) {  // 不覆盖以前的记录。/ Never overwrite old records.
        std::cerr << "记录文件 " << params.log_path << " 已经存在，为了不覆盖以前的记录，程序不运行。\n";
        return 2;
    }
    std::ofstream log(params.log_path);
    if (!log) {
        std::cerr << "打不开记录文件 " << params.log_path << "（在项目根目录运行，logs/ 要存在）\n";
        return 2;
    }

    const auto arm = kArms[params.arm];
    const auto& arm_name = record.arm;
    std::unique_ptr<rdk::Robot> robot;
    std::unique_ptr<rdk::Tool> tool;
    bool switched_to_flange = false;
    int exit_code = 0;
    try {
        robot = std::make_unique<rdk::Robot>(params.robot_sn);
        for (int sig : {SIGINT, SIGTERM, SIGHUP}) {  // 在 RDK 初始化之后装，保证由这里接管信号。/ Installed after RDK init so these handlers win.
            std::signal(sig, OnSignal);
        }
        Prepare(*robot, params);
        const auto info = robot->info();
        record.software_ver = info.software_ver;
        std::cout << "① 已连接 " << info.serial_num << "，机器人软件 " << info.software_ver << "。" << std::endl;
        robot->SwitchMode(rdk::Mode::IDLE);  // 工具的标定、新建、切换都只能在 IDLE 下。/ Tool calibration, add and switch are IDLE-only.
        tool = std::make_unique<rdk::Tool>(*robot);
        std::cout << "已有工具：";
        for (const auto& name : tool->list()) {
            std::cout << name << "  ";
        }
        std::cout << "\n当前工具：ARM_1 " << tool->name(kArms[0]) << "，ARM_2 " << tool->name(kArms[1]) << std::endl;
        record.previous_tool = tool->name(arm);
        record.previous_params = tool->params(arm);
        record.has_previous_params = true;
        PrintParams(arm_name + " 当前工具 " + record.previous_tool, record.previous_params);

        rdk::ToolParams applied;
        if (params.no_calibration) {  // 机器人不动，不切工具。/ The robot does not move and no tool is switched.
            applied.mass = params.weighed_mass;
            applied.CoM = kStlCoM;
            applied.inertia = StlInertia(params.weighed_mass);
            std::cout << "②–④ 跳过（--no-calibration）：用称重质量 + STL 质心 + STL 形状 × 称重质量。" << std::endl;
        } else {
            if (record.previous_tool != "Flange") {
                tool->Switch(arm, "Flange");
                switched_to_flange = true;
            }
            std::cout << "② " << arm_name << " 已切到 Flange（不启用任何工具）。" << std::endl;

            Ask("\n③ 装着工具标定。确认 " + params.tool_name + " 已装在 " + arm_name
                + " 法兰上。\n   按 Enter 后机器人会自己动；RDK 没说双臂时动哪条臂，两条臂周围都要清空，"
                  "手放在 E-stop 旁边。\n   按 Enter 开始，Ctrl+C 退出：");
            std::cout << "③ 标定中（装着工具），Ctrl+C 可随时停 ……" << std::endl;
            record.calibrations.push_back(CalibrateWatched(*robot, *tool, true));
            PrintParams("③ 结果", record.calibrations.back().result);

            if (!params.mounted_only) {
                const auto answer = Ask("\n④ 现在把工具从 " + arm_name
                                        + " 法兰上拆下来（可以先拍 E-stop 再拆，松开后程序会自己上伺服）。\n"
                                          "   拆好、人离开后按 Enter 开始第二次标定；输入 s 回车跳过这一步：");
                if (answer == "s" || answer == "S") {
                    std::cout << "④ 跳过。" << std::endl;
                } else {
                    std::cout << "④ 标定中（拆下工具），Ctrl+C 可随时停 ……" << std::endl;
                    record.calibrations.push_back(CalibrateWatched(*robot, *tool, false));
                    PrintParams("④ 结果", record.calibrations.back().result);
                }
            }

            PrintComparison(record.calibrations, params.weighed_mass);
            for (const auto& cali : record.calibrations) {
                if (record.not_applied_reason.empty()) {
                    record.not_applied_reason = WrongArmReason(cali, params.arm);
                }
            }
            applied = record.calibrations.back().result;  // 官方步骤 6：用最后一次的结果。/ Official step 6: the last result.
            if (params.stl_inertia) {
                applied.inertia = StlInertia(applied.mass);
            }
        }
        const bool exists = tool->exist(params.tool_name);
        // 标定结果里的 tcp_location 无效（头文件 @warning），这里另外定。/ The result's tcp_location is invalid (header @warning); set here.
        if (params.has_tcp) {
            applied.tcp_location = {params.tcp[0], params.tcp[1], params.tcp[2], 1.0, 0.0, 0.0, 0.0};
        } else if (exists) {
            applied.tcp_location = tool->params(params.tool_name).tcp_location;
        } else {
            applied.tcp_location = {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0};
        }
        if (record.not_applied_reason.empty() && !(applied.mass > 0.0)) {
            record.not_applied_reason = "标定质量 " + Fixed(applied.mass, 4) + " kg 不大于 0";
        }
        if (record.not_applied_reason.empty() && !InertiaPhysical(applied.inertia)) {
            record.not_applied_reason = "标定出的惯量在物理上不可能（主惯量非正或不满足三角不等式）；可以加 --stl-inertia 重新运行";
        }

        if (record.not_applied_reason.empty()) {
            std::cout << std::endl;
            PrintParams(std::string("将写入") + (exists ? "（更新已有工具 " : "（新建工具 ") + params.tool_name + "）",
                applied);
            if (!params.has_tcp) {
                std::cout << (exists ? "TCP 保持该工具原来的值" : "TCP 在法兰中心") << "；要改用 --tcp-mm=X,Y,Z。"
                          << std::endl;
            }
            const auto answer
                = Ask("⑤ 写入工具 " + params.tool_name + " 并在 " + arm_name + " 启用？输入 y 回车写入，直接回车不写入：");
            if (answer == "y" || answer == "Y") {
                if (exists) {
                    tool->Update(params.tool_name, applied);
                } else {
                    tool->Add(params.tool_name, applied);
                }
                tool->Switch(arm, params.tool_name);
                record.applied = true;
                record.applied_params = applied;
                PrintParams(arm_name + " 现在的工具 " + tool->name(arm), tool->params(arm));
                if (!record.calibrations.empty() && !record.calibrations.back().tool_mounted) {
                    std::cout << "工具现在是拆下来的：装回 " << arm_name << " 之后再让机器人动。" << std::endl;
                }
            } else {
                record.not_applied_reason = "操作员没有确认写入";
            }
        }
        if (!record.not_applied_reason.empty()) {
            std::cout << "没有写入工具：" << record.not_applied_reason << std::endl;
        }
    } catch (const std::exception& e) {
        record.error = e.what();
        std::cerr << "已停止：" << e.what() << std::endl;
        exit_code = 1;
    }
    if (tool && switched_to_flange && !record.applied) {  // 没写入就恢复原来的工具。/ Restore the previous tool unless the new one was saved.
        try {
            tool->Switch(arm, record.previous_tool);
            std::cout << arm_name << " 已恢复原来的工具 " << record.previous_tool << "。" << std::endl;
        } catch (const std::exception& e) {
            std::cerr << arm_name << " 没能恢复原来的工具 " << record.previous_tool << "：" << e.what() << std::endl;
        }
    }
    if (tool) {
        try {
            record.active_tool_after = tool->name(arm);
        } catch (const std::exception&) {
            // 读不到就留空。/ Left empty if it cannot be read.
        }
    }
    WriteJson(log, record);
    std::cout << "记录已写到 " << params.log_path << std::endl;
    return exit_code;
}
