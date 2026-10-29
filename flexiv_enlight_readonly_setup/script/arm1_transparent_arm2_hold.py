"""ARM_1 尽可能透明地手拖，ARM_2 全程发送 Hold 保持不动。

导纳环在控制器内部，由 FloatingCartesian 原语（末端笛卡尔空间导纳）实现；RDK 只能给它传参数。
透明度参数集中在下方"透明度参数"常量区，也可以用同名命令行参数临时覆盖。
"""

import argparse  # 解析命令行参数。
import signal  # 让 Ctrl+C、关闭终端、kill 都走同一个 Stop() 流程。
import sys  # 出错时只打印原因并以非零状态退出。
import time  # 轮询间隔与超时计时。

# ================================ 透明度参数：在这里调 ================================
# 这里只有 FloatingCartesian 原语自己的输入参数；名称、默认值、范围照抄 Flexiv Primitives 3E.1 文档
# Zero Gravity Floating 页。6 维的按末端 X Y Z Rx Ry Rz 排列，7 维的按关节 J1–J7 排列。
FLOATING_AXIS = [1] * 6  # floatingAxis：允许浮动的末端轴，1 浮动，0 锁住。文档默认 1 1 1 1 1 1。
ENABLE_ELBOW_MOTION = 1  # enableElbowMotion：肘部（零空间）能否跟着手动，1 能；0 肘部不动，末端 6 轴照样能拖。文档默认 0。设 1 肘部会自己漂（见下方 10-28 记录）。
DAMPING_LEVEL = [10.0] * 6  # dampingLevel：阻尼，越小越轻。范围 0–100，文档默认 0。0 最轻，但 FloatingCartesian 在 0 + 最小惯量下没人推也会自己动（见 10-28 记录），所以加一点让它能静止。
INERTIA_SCALE = [0.8, 0.8, 0.8, 0.65, 0.65, 0.65]  # inertiaScale：期望惯量缩放，越小越轻但可能振动。此值即下限，最轻；上限和文档默认 1。
RESPONSE_TORQUE = [0.2, 0.5, 0.2, 0.25, 0.15, 0.1, 0.2]  # responseTorque：关节死区 [Nm]，外力矩低于它推不动；越小越透明，太小会自己漂。范围 0–3，文档默认 1.5 1.5 1.5 1.5 0.5 0.5 0.3。
FRICTION_COMP_SCALE = [100.0] * 7  # frictionCompScale：摩擦补偿百分比，越大越顺。范围 50–100，文档默认 100；100 已是最顺。
MAX_VEL = 2.0  # maxVel：TCP 最大线速度 [m/s]，手推超过它会被拦住，万一跑走也最多到这个速度。范围 0.2–3.0，文档默认 3.0。
# 以下 3 个原语参数不传，用控制器默认：
#   floatingCoord：浮动参考坐标系，默认 TCP START。
#   enableSixAxisJntCtrl：1 时 A3 轴锁在初始位置不动，默认 0。
#   diEnableFloating：只有该数字输入为 ON 时才能拖动，默认 NONE（不需要信号）。
#
# 实测记录（dampingLevel 除注明外都是 0，inertiaScale、frictionCompScale 都在上面的最透明极限值）：
#   2026-10-23 responseTorque 全 0                               → ARM_1 自己加速跑走，拍了 E-stop。
#   2026-10-27 0.21 0.51 0.20 0.26 0.15 0.11 0.20（FloatingJoint）    → 不漂。
#   2026-10-27 0.19 0.42 0.09 0.18 0.10 0.08 0.14（FloatingJoint）    → 不漂；手感与上一行没有任何区别。
#   2026-10-28 0.06 0.13 0.08 0.09 0.06 0.06 0.07（FloatingCartesian，肘部 1）→ 2 秒内自己漂到 0.08 rad/s。
#   2026-10-28 0.20 0.50 0.20 0.25 0.15 0.10 0.20（FloatingCartesian，肘部 1）→ 0.4 秒内 J3、J5、J6 自己转到 0.11 rad/s，
#              J4（肘关节）不动：肘部在零空间里漂。同样 4 个参数在 FloatingJoint 下不漂，所以只把肘部改成 0。
#   2026-10-28 0.20 0.50 0.20 0.25 0.15 0.10 0.20（FloatingCartesian，肘部 0）→ 0.8 秒内 J1 自己转到 0.05 rad/s。
#              外力矩 0.10 -0.55 -0.09 0.53 -0.07 -0.10 -0.01：超死区的是 J2、J4，J1 在死区内。
#   2026-10-28 同上第二次                                         → 0.4 秒内 J2 自己转到 0.07 rad/s。
#              外力矩 -0.08 0.20 -0.05 -0.21 -0.07 -0.12 -0.04：只有 J6 超死区 0.02，J2 远在死区内；J2、J4 符号与上一次相反。
#              所以不是某个关节超死区推动的：FloatingCartesian 在这 4 个参数下自己就不能静止（同样参数 FloatingJoint 不漂）。
#   2026-10-28 同上，dampingLevel 5                                → 0.4 秒内 J2 0.06、J6 0.05 rad/s，照样漂。
#              外力矩 -0.11 0.18 -0.09 -0.21 -0.08 -0.12 -0.03，与上一次几乎一样。dampingLevel 10 尚未实测。
#   当前 responseTorque 取不漂的第一组（取整）：手感与更小的一组相同，但给随姿态变化的模型误差留了余量
#   （J2 静止误差 10-27 测到 0.41，10-28 只有 0.08）。再往下减手感不会变轻，只会更容易漂。
# ====================================================================================

# ============ 脚本自己加的参数，不是 RDK / 原语的参数：已注释掉，不再生效 ============
# RESPONSE_TORQUE = None  # None = 启动时自动测死区。只在起始姿态、两臂 Hold 时测，代表不了浮动中和换姿态后的误差，测小了就漂（10-28 那次）。
# RESPONSE_MARGIN = 0.05  # 自动死区 = 静止外力矩误差峰值 + 此余量 [Nm]。
# MEASURE_SECONDS = 2.0  # 自动死区的测量时长 [s]。
# MEASURE_PERIOD = 0.01  # 自动死区的采样周期 [s]。
# 对应的 --response-margin 命令行参数和测量代码已删除；responseTorque 直接用上面的 RESPONSE_TORQUE。
#
# 下面几个也不是 RDK 参数，但保留：它们不发给机器人、不影响手感，只做安全检查和程序流程。
STILL_SECONDS = 2.0  # 安全：开始浮动后这段时间内不要碰 ARM_1；期间任一臂自己动起来就整机停 [s]。设 0 关掉 ARM_1 的检查。
STILL_JOINT_VEL = 0.05  # 安全：判定"没人推却在动"的关节速度阈值 [rad/s]；ARM_2 全程都按它检查。
OPERATIONAL_TIMEOUT = 30.0  # 流程：上伺服后等待就绪的最长时间 [s]。
CONFIRM_TIMEOUT = 2.0  # 流程：下发后确认两臂原语都在运行的最长时间 [s]。
POLL_PERIOD = 0.05  # 流程：状态监控周期 [s]。
FLOATING_PRIMITIVE = "FloatingCartesian"  # ARM_1 运行的原语。
# 回正：浮动前 ARM_1 先用 Home 原语回到控制器的 home 姿态（文档默认 0 -40 0 90 0 -50 0 deg），ARM_2 照样 Hold。
# 命令行 --no-home 跳过回正，直接在当前姿态浮动。
HOME_JNT_VEL_SCALE = 20  # Home 原语的 jntVelScale：关节速度百分比，100 为全速。范围 1–100，文档默认 20。不影响手感。
HOME_TIMEOUT = 30.0  # 流程：回正到位并停稳的最长时间 [s]。

VECTOR_LIMITS = {  # 文档给出的范围：原语参数名 -> (下限, 上限)。
    "floatingAxis": ([0] * 6, [1] * 6),
    "dampingLevel": ([0.0] * 6, [100.0] * 6),
    "inertiaScale": ([0.8, 0.8, 0.8, 0.65, 0.65, 0.65], [1.0] * 6),
    "responseTorque": ([0.0] * 7, [3.0] * 7),
    "frictionCompScale": ([50.0] * 7, [100.0] * 7),
}
MAX_VEL_LIMITS = (0.2, 3.0)  # maxVel 的范围 [m/s]。


def floating_params(args):  # 按原语参数名组织，直接作为 FloatingCartesian 的输入。
    return {
        "floatingAxis": args.floating_axis,
        "enableElbowMotion": args.enable_elbow_motion,
        "dampingLevel": args.damping_level,
        "inertiaScale": args.inertia_scale,
        "responseTorque": args.response_torque,
        "frictionCompScale": args.friction_comp_scale,
        "maxVel": args.max_vel,
    }


def parse_args(argv=None):  # 读取命令行；未给出的参数用上方常量。
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("robot_sn", help="完整序列号，例如 Enlight-LL-062034")
    cart = dict(nargs=6, type=float, metavar="A")  # 末端 X Y Z Rx Ry Rz 各一个值。
    joint = dict(nargs=7, type=float, metavar="J")  # 关节 J1–J7 各一个值。
    parser.add_argument("--floating-axis", default=FLOATING_AXIS, nargs=6, type=int, metavar="A")
    parser.add_argument("--enable-elbow-motion", default=ENABLE_ELBOW_MOTION, type=int, choices=(0, 1))
    parser.add_argument("--damping-level", default=DAMPING_LEVEL, **cart)
    parser.add_argument("--inertia-scale", default=INERTIA_SCALE, **cart)
    parser.add_argument("--response-torque", default=RESPONSE_TORQUE, **joint)
    parser.add_argument("--friction-comp-scale", default=FRICTION_COMP_SCALE, **joint)
    parser.add_argument("--max-vel", type=float, default=MAX_VEL)
    parser.add_argument("--no-home", dest="home", action="store_false", help="不先回正，直接在当前姿态浮动")
    parser.add_argument("--clear-fault", action="store_true", help="启动时若有故障先尝试清除；不加则直接退出")
    args = parser.parse_args(argv)

    params = floating_params(args)
    for name, (lower, upper) in VECTOR_LIMITS.items():  # 下发前按文档范围检查，NaN 也会被拒绝。
        for index, (value, lo, hi) in enumerate(zip(params[name], lower, upper), start=1):
            if not lo <= value <= hi:
                parser.error(f"{name} 第 {index} 个值 {value} 超出范围 [{lo}, {hi}]")
    lo, hi = MAX_VEL_LIMITS
    if not lo <= args.max_vel <= hi:
        parser.error(f"maxVel={args.max_vel} 超出范围 [{lo}, {hi}]")
    return args


def build_commands(flexivrdk, arm1_command):  # 两臂放进同一条命令；ARM_2 永远是 Hold。
    commands = {
        flexivrdk.JointGroup.ARM_1: arm1_command,
        flexivrdk.JointGroup.ARM_2: flexivrdk.PrimitiveArgs("Hold", {}),  # Hold 没有输入参数。
    }
    for command in commands.values():
        command.external_axis_control = False  # 不带动外部轴。
        command.sync_motion_mode = flexivrdk.SyncMotionMode.DISABLE  # 两臂互不跟随。
    return commands


def not_running_reason(robot, flexivrdk):  # 机器人不能继续运行的具体原因；正常时返回 None。
    if not robot.connected():
        return "与机器人断线"
    if not robot.estop_released():
        return "E-stop 被按下"
    if robot.fault():
        return "机器人故障"
    if not robot.operational():
        return f"机器人未就绪：{robot.operational_status()}"
    if robot.mode() != flexivrdk.Mode.NRT_PRIMITIVE_EXECUTION:
        return f"控制模式被切到 {robot.mode()}"
    return None


def check_still(robot_states, group, flexivrdk, args):  # 没人推却在动就抛出，附上外力矩和提示。
    speeds = [abs(v) for v in robot_states[group].dtheta]  # 电机侧关节速度，噪声小。
    if max(speeds) <= STILL_JOINT_VEL:
        return
    moving = "、".join(f"J{i} {s:.2f}" for i, s in enumerate(speeds, start=1) if s > STILL_JOINT_VEL)
    tau_ext = robot_states[group].tau_ext  # 对比死区：哪个关节超过了它。
    if group != flexivrdk.JointGroup.ARM_1:
        hint = "Hold 没有保持住"
    elif args.enable_elbow_motion:
        hint = "肘部（零空间）在漂，把 ENABLE_ELBOW_MOTION 设为 0"
    else:  # 这是臂已经在动时的外力矩，只作记录，不代表是哪个关节推动的。
        over = "、".join(f"J{i} {abs(t):.2f}>{r:g}" for i, (t, r)
                        in enumerate(zip(tau_ext, args.response_torque), start=1) if abs(t) > r)
        hint = (f"阻尼不够，调大 DAMPING_LEVEL（现在 {args.damping_level}）；"
                + (f"超过 RESPONSE_TORQUE 的关节：{over}" if over else "外力矩都在 RESPONSE_TORQUE 以内"))
    raise RuntimeError(f"{flexivrdk.kJointGroupNames[group]} 没人推却在动"
                       f"（{moving} rad/s；外力矩 {[round(t, 2) for t in tau_ext]} Nm）：{hint}")


def home_arm1(robot, flexivrdk, args):  # ARM_1 用 Home 原语回正，ARM_2 照样 Hold；到位并停稳后返回。
    arm1 = flexivrdk.JointGroup.ARM_1
    robot.ExecutePrimitive(build_commands(flexivrdk, flexivrdk.PrimitiveArgs("Home", {"jntVelScale": HOME_JNT_VEL_SCALE})))
    print(f"ARM_1 回正中（Home，关节速度 {HOME_JNT_VEL_SCALE}%），Ctrl+C 可随时停 ……", flush=True)
    deadline = time.monotonic() + HOME_TIMEOUT
    while True:
        reason = not_running_reason(robot, flexivrdk)
        if reason:
            raise RuntimeError(reason)
        robot_states = robot.states()
        check_still(robot_states, flexivrdk.JointGroup.ARM_2, flexivrdk, args)  # 回正期间 ARM_2 也不许动。
        state = robot.primitive_states().get(arm1)
        reached = state is not None and state.pt_name == "Home" and state.names_and_values.get("reachedTarget", 0)
        if reached and max(abs(v) for v in robot_states[arm1].dtheta) <= STILL_JOINT_VEL:
            print("ARM_1 已回正。", flush=True)
            return
        if time.monotonic() > deadline:
            raise RuntimeError(f"ARM_1 {HOME_TIMEOUT:g} 秒内没有回正到位并停稳")
        time.sleep(POLL_PERIOD)


def main(argv=None):
    args = parse_args(argv)

    import flexivrdk  # 延迟导入 SDK，使 --help 和参数检查不需要加载 SDK。

    def interrupt(*_):  # 把终止信号统一转成 KeyboardInterrupt，保证走到 finally 里的 Stop()。
        raise KeyboardInterrupt

    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, interrupt)

    groups = (flexivrdk.JointGroup.ARM_1, flexivrdk.JointGroup.ARM_2)
    robot = flexivrdk.Robot(args.robot_sn)  # 连接机器人。
    if robot.fault():
        if not args.clear_fault:
            raise RuntimeError("机器人有故障；确认原因后加 --clear-fault 重试")
        if not robot.ClearFault():
            raise RuntimeError("故障无法清除")
    missing = [flexivrdk.kJointGroupNames[g] for g in groups if g not in robot.info().single_arm_groups]
    if missing:
        raise RuntimeError(f"控制器没有这些单臂关节组：{missing}")
    if not robot.all_stopped():  # 有别的程序在动机器人时不接管。
        raise RuntimeError("机器人还在运动；先停止其他控制程序")
    if not robot.operational():
        robot.ServoOn()  # 上伺服；E-stop 必须已松开。
        deadline = time.monotonic() + OPERATIONAL_TIMEOUT
        while not robot.operational():
            if time.monotonic() > deadline:
                raise RuntimeError("等待机器人就绪超时（检查 E-stop、AUTO(Remote)）")
            time.sleep(0.5)

    try:
        robot.SwitchMode(flexivrdk.Mode.NRT_PRIMITIVE_EXECUTION)
        if args.home:
            home_arm1(robot, flexivrdk, args)
        params = floating_params(args)
        commands = build_commands(flexivrdk, flexivrdk.PrimitiveArgs(FLOATING_PRIMITIVE, params))
        expected = {group: command.pt_name for group, command in commands.items()}  # 每条臂应在运行的原语。
        robot.ExecutePrimitive(commands)  # 阻塞到两条原语开始执行。
        print(f"ARM_1 {FLOATING_PRIMITIVE} 参数：{params}", flush=True)
        print(f"静止检查 {STILL_SECONDS:g} 秒，先不要碰 ARM_1 ……", flush=True)
        started, confirmed, free = time.monotonic(), False, False
        while True:
            reason = not_running_reason(robot, flexivrdk)
            if reason:
                raise RuntimeError(reason)
            states = robot.primitive_states()  # 只包含正在运行原语的关节组。
            running = all(
                group in states and states[group].pt_name == name
                and not states[group].names_and_values.get("terminated", 0)
                for group, name in expected.items()
            )
            if running:
                confirmed = True
            elif confirmed or time.monotonic() - started > CONFIRM_TIMEOUT:
                actual = {
                    flexivrdk.kJointGroupNames[g]: s.pt_name + ("(已终止)" if s.names_and_values.get("terminated", 0) else "")
                    for g, s in states.items()
                }
                raise RuntimeError(f"ARM_1 {FLOATING_PRIMITIVE} / ARM_2 Hold 不在运行，实际：{actual}")

            still_check = time.monotonic() - started < STILL_SECONDS
            robot_states = robot.states()
            for group in expected:
                if group == flexivrdk.JointGroup.ARM_1 and not still_check:
                    continue  # 静止检查结束后 ARM_1 由人手拖，不再限速。
                check_still(robot_states, group, flexivrdk, args)
            if confirmed and not still_check and not free:
                print("静止检查通过：ARM_1 可以手拖；ARM_2 Hold 不动。按 Ctrl+C 结束。", flush=True)
                free = True
            time.sleep(POLL_PERIOD)
    except KeyboardInterrupt:
        pass
    finally:
        for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):  # 停机过程中不再被信号打断。
            signal.signal(sig, signal.SIG_IGN)
        try:
            robot.Stop()  # 整机停到 IDLE，两臂都保持当前位置。
            print("已 Stop()：机器人进入 IDLE 并保持当前位置。", flush=True)
        except RuntimeError as error:  # E-stop 按下时 Stop() 无法执行，机器人已由 E-stop 停住。
            print(f"Stop() 没有执行：{error}", flush=True)


if __name__ == "__main__":
    try:
        main()
    except RuntimeError as error:  # 只打印原因，不打印 Python 调用栈。
        sys.exit(f"已停止：{error}")
