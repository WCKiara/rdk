#!/usr/bin/env python
"""单臂笛卡尔位置控制：斜线运动、工具倾斜、软件位置边界、独立回 Home。

本地定制版：根据工具倾斜并沿斜线移动的需求，将原示例改为纯位置控制。
默认仅操作 ARM_1，取消自动 Home、双臂搜索接触、Z 轴恒力下压和无限循环。
--home-only 仅执行所选手臂的预设 Home 原语，不接着执行斜向运动。
--return-home 在斜向运动成功到位后停止，再让同一条手臂回预设 Home，最后停止。
中断、故障、越界或运动超时时直接停止，不启动回 Home。
--clear-minor-fault 仅在启动前尝试清除一次 MINOR_FAULT；运行中不自动清故障。
--delta/--position 指定世界坐标系的位置目标；--tilt/--rpy 指定目标工具姿态。
--motion-method movel 使用非实时 MoveL 原语，控制器规划直线及姿态变化。
--motion-method nrt 使用直接非实时控制，只发送一次最终目标，不连续发送微小中间目标。
两种方式都由控制器生成轨迹；frequency 只控制状态/边界监测频率。
duration 是预计时间/超时预算，不保证控制器运动严格持续该时长。

position limit 是世界坐标系 TCP 的软件包围盒：目标先检查，实测位置持续检查。
它不限制整条手臂/工具外形，不保证碰撞避免，也不是控制器的硬限位。
Home 是关节运动，不能预先证明 TCP 全路径在包围盒内，只能在线监测并停止。
边界监测和 Stop 存在通信、采样与制动延迟，应按现场情况保留距离。
SDK 的 ServoOn、SwitchMode、Stop 是机器人级操作；运动目标仅发给所选手臂。
"""

__copyright__ = "Copyright (C) 2016-2026 Flexiv Ltd. All Rights Reserved."
__author__ = "Flexiv"  # 原示例作者；本文件已作本地定制。

import argparse
import logging
import math
import sys
import time
from dataclasses import dataclass

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

# 默认操作手臂；不能根据 ARM_1/ARM_2 名称自行推断现场左右手。
DEFAULT_ARM = "ARM_1"
# 临时软件位置范围：根据本次 ARM_1 当前位置及先前 Home 记录设定，单位 m。
# 不是厂家推荐范围；更换手臂/工具/场地时，应通过命令行指定实际需要的范围。
Z_BEFORE_HOME = -0.026941440999507904  # 本次单臂回 Home 前的世界 Z，m。
Z_AT_LIMIT_UPDATE = 0.12149951606988907  # 回 Home 后更新限位时只读实测的世界 Z，m。
Z_MIDPOINT = (Z_BEFORE_HOME + Z_AT_LIMIT_UPDATE) / 2.0  # 两次 Z 的中点，作为固定 Z 下限。
# 中点只在此次限位设置时计算；不会随以后每次启动时的当前位置重新计算或下移。
POSITION_MIN = [0.35, 0.15, Z_MIDPOINT]  # [X最小值, Y最小值, Z最小值]，世界坐标系。
POSITION_MAX = [0.46, 0.25, 0.18]  # [X最大值, Y最大值, Z最大值]，世界坐标系。
LIMIT_MARGIN = 0.005  # 距离包围盒边界的内缩余量，m；进入余量区即停止。
MAX_CONTACT_WRENCH = [10.0, 10.0, 10.0, 3.0, 3.0, 3.0]  # [Fx,Fy,Fz,Mx,My,Mz]，N、N·m。


def finite_vector(values, size, name):
    """values：输入数组；size：要求的长度；name：错误信息使用的参数名。"""
    result = np.asarray(values, dtype=float)
    if result.shape != (size,) or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} 必须是 {size} 个有限数值")
    return result


def pose_rotation(pose):
    """pose：SDK 世界系 [x,y,z,qw,qx,qy,qz]；转换为 SciPy 的 [qx,qy,qz,qw]。"""
    pose = finite_vector(pose, 7, "TCP 位姿")
    quat = pose[[4, 5, 6, 3]]
    if np.linalg.norm(quat) < 1e-8:
        raise ValueError("TCP 四元数不能为零")
    return Rotation.from_quat(quat)


@dataclass
class PositionLimits:
    """lower/upper：世界系 XYZ 外边界，m；margin：向内部预留的距离，m。"""
    lower: np.ndarray
    upper: np.ndarray
    margin: float

    def __post_init__(self):
        self.lower = finite_vector(self.lower, 3, "position-min")
        self.upper = finite_vector(self.upper, 3, "position-max")
        if not math.isfinite(self.margin) or self.margin < 0:
            raise ValueError("limit-margin 必须为非负有限数")
        if np.any(self.upper - self.lower <= 2 * self.margin):
            raise ValueError("各轴上限必须大于下限，且范围宽度必须大于两倍 limit-margin")

    def check(self, position, name):
        """position：待检查 XYZ，m；name：起点/目标/实测等说明。越界拒绝，不静默截断。"""
        position = finite_vector(position, 3, name)
        if np.any(position < self.lower + self.margin) or np.any(position > self.upper - self.margin):
            raise ValueError(f"{name} {position.tolist()} 超出含内缩余量的位置边界")


@dataclass
class MotionPlan:
    """start/target：七维 TCP 位姿；duration：参考轨迹时长/等待预算 s；limits：软件边界。"""
    start: np.ndarray
    target: np.ndarray
    duration: float
    limits: PositionLimits

    def __post_init__(self):
        # rotation_path：姿态球面插值；输入顺序转换后交给 SciPy，不直接插值欧拉角。
        self.rotation_path = Slerp([0.0, 1.0], Rotation.concatenate([
            pose_rotation(self.start), pose_rotation(self.target)
        ]))

    def sample(self, elapsed):
        """elapsed：离线参考时间 s；仅用于规划验证，不把这些中间目标循环发送给 NRT。"""
        u = min(max(elapsed / self.duration, 0.0), 1.0)
        # s：五次平滑进度，起止速度与加速度均为零，避免直接跳到最终目标。
        s = u**3 * (10.0 - 15.0 * u + 6.0 * u**2)
        xyz = self.start[:3] + s * (self.target[:3] - self.start[:3])
        qx, qy, qz, qw = self.rotation_path(s).as_quat()
        self.limits.check(xyz, "轨迹目标")
        return [*xyz.tolist(), qw, qx, qy, qz]


def build_plan(start_pose, args, limits):
    """start_pose：当前位姿；args：命令行运动参数；limits：在发运动命令前生效的边界。"""
    start = finite_vector(start_pose, 7, "起始 TCP 位姿").copy()
    start_rotation = pose_rotation(start)
    limits.check(start[:3], "起始位置")
    target = start.copy()
    # --delta 按世界 X/Y/Z 分别相对位移；--position 是世界系绝对 XYZ，单位 m。
    target[:3] = args.position if args.position is not None else start[:3] + np.array(args.delta)
    if args.rpy is not None:
        # --rpy：世界系绝对欧拉角，度；固定轴依次 X、Y、Z，R=Rz*Ry*Rx。
        target_rotation = Rotation.from_euler("xyz", args.rpy, degrees=True)
    elif args.tilt is not None:
        # --tilt：相对起始工具坐标系的旋转增量，度；R目标=R起点*R增量。
        # 增量内部也采用固定轴 xyz 顺序；仅填一个轴可明确指定绕该起始工具轴倾斜。
        target_rotation = start_rotation * Rotation.from_euler("xyz", args.tilt, degrees=True)
    else:
        target_rotation = start_rotation  # 未指定姿态参数时保持当前姿态。
    qx, qy, qz, qw = target_rotation.as_quat()
    target[3:] = [qw, qx, qy, qz]
    limits.check(target[:3], "最终目标位置")
    # 包围盒是凸集，起点/终点均合规时，二者之间所有直线插值目标也合规。
    distance = np.linalg.norm(target[:3] - start[:3])  # 总位移长度，m。
    angle = (start_rotation.inv() * target_rotation).magnitude()  # 最短姿态转角，rad。
    # 五次进度函数的最大一阶/二阶导数分别为 1.875、10/sqrt(3)。
    # 时长不足时自动延长，以同时满足位置和姿态的速度/加速度设置。
    peak_acc = 10.0 / math.sqrt(3.0)
    duration = max(args.duration, 1.875 * distance / args.max_linear_vel,
                   1.875 * angle / args.max_angular_vel,
                   math.sqrt(peak_acc * distance / args.max_linear_acc),
                   math.sqrt(peak_acc * angle / args.max_angular_acc))
    return MotionPlan(start, target, duration, limits)


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("robot_sn", help="目标机器人完整序列号，不含空格")
    parser.add_argument("frequency", type=int, help="状态/位置边界监测的名义频率，1~100 Hz；移动目标只发一次")
    parser.add_argument("--arm", choices=["ARM_1", "ARM_2"], default=DEFAULT_ARM,
                        help="唯一运动手臂，默认 ARM_1；全部运动/原语字典只包含该组")
    action = parser.add_mutually_exclusive_group(required=True)
    action.add_argument("--home-only", action="store_true", help="仅回该手臂的预设 Home，完成后停止")
    action.add_argument("--delta", nargs=3, type=float, metavar=("DX", "DY", "DZ"),
                        help="相对当前位置的世界系位移，m，例如 0.01 0 -0.01 为 XZ 平面斜线")
    action.add_argument("--position", nargs=3, type=float, metavar=("X", "Y", "Z"),
                        help="世界系绝对目标位置，m，与 --delta 二选一")
    parser.add_argument("--return-home", action="store_true",
                        help="斜向运动成功到位后让同一手臂回预设 Home，随后停止；默认关闭")
    parser.add_argument("--motion-method", choices=["nrt", "movel"], default="nrt",
                        help="nrt=直接非实时控制；movel=非实时 MoveL 原语；均只发送一次最终目标")
    parser.add_argument("--movel-acc", type=float, default=0.1,
                        help="MoveL 的 acc，单位 m/s²，有效范围 0.1~3.0；与直接 NRT 的线加速度分开")
    orientation = parser.add_mutually_exclusive_group()
    orientation.add_argument("--tilt", nargs=3, type=float, metavar=("RX", "RY", "RZ"),
                             help="相对起始工具坐标系的旋转增量，度，例如 0 15 0")
    orientation.add_argument("--rpy", nargs=3, type=float, metavar=("RX", "RY", "RZ"),
                             help="世界系绝对姿态欧拉角，度，固定轴 xyz；不与 --tilt 混用")
    parser.add_argument("--position-min", nargs=3, type=float, default=POSITION_MIN,
                        help=f"世界系 [Xmin,Ymin,Zmin]，m；默认 {POSITION_MIN}")
    parser.add_argument("--position-max", nargs=3, type=float, default=POSITION_MAX,
                        help="世界系 [Xmax,Ymax,Zmax]，m；默认 0.46 0.25 0.18")
    parser.add_argument("--limit-margin", type=float, default=LIMIT_MARGIN,
                        help="边界内缩距离，m；默认 0.005，不是硬限位的制动距离保证")
    parser.add_argument("--duration", type=float, default=3.0,
                        help="预计运动时间/等待预算，s；实际时间由控制器生成器决定，不保证严格时长")
    parser.add_argument("--max-linear-vel", type=float, default=0.01,
                        help="笛卡尔最大线速度，m/s；默认 0.01 = 10 mm/s")
    parser.add_argument("--max-angular-vel", type=float, default=0.15,
                        help="笛卡尔最大角速度，rad/s；MoveL 自动转成 angVel 的度/s")
    parser.add_argument("--max-linear-acc", type=float, default=0.05,
                        help="直接 NRT 的最大线加速度，m/s²；MoveL 的加速度用 --movel-acc")
    parser.add_argument("--max-angular-acc", type=float, default=0.3,
                        help="直接 NRT 的最大角加速度，rad/s²；MoveL 使用原语内部角加速度设置")
    parser.add_argument("--max-contact-wrench", nargs=6, type=float, default=MAX_CONTACT_WRENCH,
                        help="[Fx,Fy,Fz,Mx,My,Mz] 上限，N、N·m；NRT 为控制器调节，MoveL 为软件监测超限停止")
    parser.add_argument("--position-tolerance", type=float, default=0.001,
                        help="结束时的实测位置误差容限，m；默认 1 mm")
    parser.add_argument("--angle-tolerance", type=float, default=1.0,
                        help="结束时的实测姿态角误差容限，度；默认 1 度")
    parser.add_argument("--startup-timeout", type=float, default=10.0,
                        help="等待伺服就绪的最长时间，s")
    parser.add_argument("--finish-timeout", type=float, default=5.0,
                        help="规划时长结束后等待跟踪到位的最长时间，s")
    parser.add_argument("--home-timeout", type=float, default=30.0,
                        help="Home 原语到位的最长时间，s；超时停止")
    parser.add_argument("--clear-minor-fault", action="store_true",
                        help="启动前仅对 MINOR_FAULT 尝试清故障一次；默认关闭，不处理严重故障")
    parser.add_argument("--fault-timeout", type=int, default=5,
                        help="启动前清轻微故障的超时，整数 s，默认 5；不影响运动超时")
    parser.add_argument("--preview", action="store_true",
                        help="只连接、读取当前位置、检查并打印目标/边界；不发伺服、模式或运动命令")
    return parser


def validate_args(args):
    """args：命令行参数。所有静态参数先校验，再允许建立机器人连接。"""
    if not 1 <= args.frequency <= 100:
        raise ValueError("frequency 必须在 1~100 Hz")
    if not args.robot_sn.strip() or any(c.isspace() for c in args.robot_sn):
        raise ValueError("robot_sn 不能为空或含空白")
    for name in ("duration", "max_linear_vel", "max_angular_vel", "max_linear_acc",
                 "max_angular_acc", "position_tolerance", "angle_tolerance",
                 "startup_timeout", "finish_timeout", "home_timeout"):
        value = getattr(args, name)
        if not math.isfinite(value) or value <= 0:
            raise ValueError(f"{name} 必须为正的有限数")
    for name in ("delta", "position", "tilt", "rpy"):
        if getattr(args, name) is not None:
            finite_vector(getattr(args, name), 3, name)
    wrench = finite_vector(args.max_contact_wrench, 6, "max-contact-wrench")
    if np.any(wrench <= 0):
        raise ValueError("max-contact-wrench 各项必须为正数")
    if args.home_only and (args.tilt is not None or args.rpy is not None):
        raise ValueError("--home-only 不接受倾斜/姿态参数；Home 使用机器人的预设姿态")
    if args.home_only and args.return_home:
        raise ValueError("--return-home 用于 --delta/--position，不与 --home-only 同用")
    if args.fault_timeout <= 0:
        raise ValueError("fault-timeout 必须为正整数")
    if args.preview and args.clear_minor_fault:
        raise ValueError("--preview 只读，不与 --clear-minor-fault 同用")
    if args.motion_method == "movel" and not args.home_only:
        if not 0.001 <= args.max_linear_vel <= 2.2:
            raise ValueError("MoveL 的 max-linear-vel 必须在 0.001~2.2 m/s")
        if not 1.0 <= math.degrees(args.max_angular_vel) <= 500.0:
            raise ValueError("MoveL 的 max-angular-vel 转换后必须在 1~500 度/s")
        if not math.isfinite(args.movel_acc) or not 0.1 <= args.movel_acc <= 3.0:
            raise ValueError("movel-acc 必须在 0.1~3.0 m/s²")
    return PositionLimits(args.position_min, args.position_max, args.limit_margin)


def clear_minor_fault(robot, sdk, logger, timeout):
    """仅清已停止且处于 IDLE 的机器人的轻微故障；timeout 为 SDK 请求超时，整数 s。"""
    if not robot.connected():
        raise RuntimeError("机器人连接已断开，无法清除故障")
    if not robot.fault():
        return
    status = robot.operational_status()
    if status != sdk.OperationalStatus.MINOR_FAULT:
        raise RuntimeError(f"启动状态为 {status}；仅允许恢复 MINOR_FAULT，请在 Elements 查看故障详情")
    if robot.mode() != sdk.Mode.IDLE or not robot.all_stopped():
        raise RuntimeError("清轻微故障前机器人必须处于 IDLE 且全部关节组停止")
    logger.warning("检测到 MINOR_FAULT，启动前仅尝试清除一次，超时 %s s", timeout)
    if not robot.ClearFault(timeout):
        raise RuntimeError("轻微故障未能清除，请在 Elements 查看当前故障详情")
    if not robot.connected() or robot.fault():
        raise RuntimeError("ClearFault 返回后仍断连或有故障，取消后续运动")
    logger.info("MINOR_FAULT_CLEARED：connected=%s fault=%s status=%s",
                robot.connected(), robot.fault(), robot.operational_status())


def checked_pose(robot, group, limits, max_wrench=None):
    """group：唯一手臂；limits：TCP 边界；max_wrench：可选六维软件接触阈值，N、N·m。"""
    if not robot.connected():
        raise RuntimeError("机器人连接已断开")
    if robot.fault():
        raise RuntimeError(f"机器人报告故障：{robot.operational_status()}；请在 Elements 查看故障详情。"
                           "仅启动时的 MINOR_FAULT 可用 --clear-minor-fault 尝试恢复")
    states = robot.states()[group]
    pose = finite_vector(states.tcp_pose, 7, "实测 TCP 位姿")
    pose_rotation(pose)
    limits.check(pose[:3], "实测 TCP 位置")
    if max_wrench is not None:
        wrench = finite_vector(states.tcp_wrench, 6, "实测 TCP 力/力矩")
        if np.any(np.abs(wrench) > np.asarray(max_wrench)):
            raise RuntimeError(f"实测 TCP 力/力矩 {wrench.tolist()} 超过软件接触阈值")
    return pose


def home_with_limits(robot, group, sdk, args, limits, logger):
    """仅发 Home 原语到 group；用非阻塞启动后的轮询持续检查边界和到位状态。"""
    robot.SwitchMode(sdk.Mode.NRT_PRIMITIVE_EXECUTION)
    checked_pose(robot, group, limits)  # 模式切换后、启动 Home 前再次检查。
    logger.info("仅让 %s 回预设 Home；关节轨迹的 TCP 位置将持续接受边界检查", args.arm)
    # {}：不覆盖 Home 的预设关节目标或原语默认参数；不用 Home() 的空 groups 默认值。
    # block_until_started=True：等待本次原语真正启动，避免误用上一次的到位状态。
    # 这不是等待整段 Home 完成；启动请求通常约 200 ms，期间无法作 Python 轮询。
    robot.ExecutePrimitive({group: sdk.PrimitiveArgs("Home", {})}, True)
    deadline = time.monotonic() + args.home_timeout
    while True:
        pose = checked_pose(robot, group, limits)
        states = robot.primitive_states().get(group)
        values = {} if states is None or states.pt_name != "Home" else states.names_and_values
        if values.get("reachedTarget") in (True, 1):
            logger.info("HOME_REACHED：%s TCP=%s", args.arm, pose.tolist())
            return
        if values.get("terminated") in (True, 1):
            raise RuntimeError("Home 已终止但没有确认 reachedTarget")
        if time.monotonic() >= deadline:
            raise TimeoutError("Home 等待到位超时")
        time.sleep(1.0 / args.frequency)  # 轮询间隔，s；SDK 调用耗时会增加实际周期。


def move_with_limits(robot, group, sdk, args, limits, plan, logger):
    """直接 NRT：只发一次最终目标，由控制器生成轨迹；循环只监测，不反复重新规划。"""
    robot.SwitchMode(sdk.Mode.NRT_CARTESIAN_MOTION_FORCE)
    robot.SetForceControlFrame(group, sdk.CoordType.WORLD)  # 位置目标始终使用世界坐标系。
    robot.SetForceControlAxis(group, [False] * 6)  # 六轴纯运动控制，使位置和姿态均可指定。
    robot.SetMaxContactWrench(group, args.max_contact_wrench)  # 有限接触上限，不设为 inf。
    checked_pose(robot, group, limits)
    command = sdk.NrtCartesianCmd(
        plan.target.tolist(),  # pose_d：最终 [x,y,z,qw,qx,qy,qz]，世界系，位置 m。
        [0.0] * 6,  # wrench_d：六维目标力/力矩，全零且不启用力控。
        [0.0] * 6,  # twist_d：末端目标速度全零，由内部运动生成器规划启停。
        args.max_linear_vel,  # max_linear_vel：最大线速度，m/s。
        args.max_angular_vel,  # max_angular_vel：最大角速度，rad/s。
        args.max_linear_acc,  # max_linear_acc：最大线加速度，m/s²。
        args.max_angular_acc,  # max_angular_acc：最大角加速度，rad/s²。
    )
    robot.SendCartesianMotionForce({group: command})
    logger.info("NRT_TARGET_SENT：只发送一次最终目标，之后仅监测")
    start_time = time.monotonic()
    target_rotation = pose_rotation(plan.target)
    while True:
        measured = checked_pose(robot, group, limits)
        elapsed = time.monotonic() - start_time
        distance_error = np.linalg.norm(measured[:3] - plan.target[:3])  # m。
        angle_error = (pose_rotation(measured).inv() * target_rotation).magnitude()  # rad。
        if distance_error <= args.position_tolerance and angle_error <= math.radians(args.angle_tolerance):
            logger.info("TARGET_REACHED：位置误差 %.6f m，姿态误差 %.3f 度",
                        distance_error, math.degrees(angle_error))
            return
        if elapsed >= plan.duration + args.finish_timeout:
            raise TimeoutError("未在规定时间内到达目标位置及姿态")
        time.sleep(1.0 / args.frequency)


def movel_with_limits(robot, group, sdk, args, limits, plan, logger):
    """非实时 MoveL：一次发送目标，使用该原语的 reachedTarget 和实测误差确认到位。"""
    # Coord 的位置单位 m、姿态单位度，参考系为 WORLD/WORLD_ORIGIN。
    target_rpy = pose_rotation(plan.target).as_euler("xyz", degrees=True).tolist()
    target = sdk.Coord(plan.target[:3].tolist(), target_rpy, ["WORLD", "WORLD_ORIGIN"])
    parameters = {
        "target": target,  # target：最终 TCP 位置及姿态，不发送中间小段目标。
        "vel": args.max_linear_vel,  # vel：TCP 线速度，m/s，有效范围 0.001~2.2。
        "angVel": math.degrees(args.max_angular_vel),  # angVel：TCP 角速度，度/s，范围 1~500。
        "acc": args.movel_acc,  # acc：线加速度，m/s²，范围 0.1~3.0。
        "zoneRadius": "ZFine",  # 不与下一段混合，在目标点精确停止。
    }
    robot.SwitchMode(sdk.Mode.NRT_PRIMITIVE_EXECUTION)
    checked_pose(robot, group, limits, args.max_contact_wrench)
    # SetMaxContactWrench 不适用于此模式，因此使用 checked_pose 中的实测力/力矩监测。
    # True 仅等待原语开始；随后在轮询中检查边界，不阻塞等待整个轨迹完成。
    robot.ExecutePrimitive({group: sdk.PrimitiveArgs("MoveL", parameters)}, True)
    logger.info("MOVEL_TARGET_SENT：一次发送非实时 MoveL 目标，之后仅监测")
    deadline = time.monotonic() + plan.duration + args.finish_timeout
    target_rotation = pose_rotation(plan.target)
    while True:
        measured = checked_pose(robot, group, limits, args.max_contact_wrench)
        states = robot.primitive_states().get(group)
        values = {} if states is None or states.pt_name != "MoveL" else states.names_and_values
        distance_error = np.linalg.norm(measured[:3] - plan.target[:3])
        angle_error = (pose_rotation(measured).inv() * target_rotation).magnitude()
        if (values.get("reachedTarget") in (True, 1)
                and distance_error <= args.position_tolerance
                and angle_error <= math.radians(args.angle_tolerance)):
            logger.info("MOVEL_TARGET_REACHED：位置误差 %.6f m，姿态误差 %.3f 度",
                        distance_error, math.degrees(angle_error))
            return
        if values.get("terminated") in (True, 1) and values.get("reachedTarget") not in (True, 1):
            raise RuntimeError("MoveL 已终止但没有确认 reachedTarget")
        if time.monotonic() >= deadline:
            raise TimeoutError("MoveL 未在规定时间内到达目标位置及姿态")
        time.sleep(1.0 / args.frequency)


def run(args, sdk=None, logger=None):
    """args：启动参数；sdk/logger：可注入模拟接口与日志，供无硬件测试使用。"""
    limits = validate_args(args)
    if sdk is None:
        import flexivrdk as sdk  # 只在实际运行/预览时加载 SDK；--help 和规划测试不连接硬件。
    logger = logger or logging.getLogger("SingleArmCartesian")
    robot = sdk.Robot(args.robot_sn)
    group = getattr(sdk.JointGroup, args.arm)
    control_started = False  # 只有开始改变控制状态后，退出时才发送 Stop。
    try:
        if group not in robot.info().single_arm_groups:
            raise ValueError(f"机器人没有 {args.arm} 单臂关节组")
        if robot.mode() != sdk.Mode.IDLE or not robot.all_stopped():
            raise RuntimeError("启动前机器人必须处于 IDLE 且全部关节组停止，避免打断其他任务")
        if args.clear_minor_fault:
            clear_minor_fault(robot, sdk, logger, args.fault_timeout)
        start_pose = checked_pose(robot, group, limits)
        plan = None if args.home_only else build_plan(start_pose, args, limits)
        logger.info("手臂=%s，当前 TCP=%s", args.arm, start_pose.tolist())
        logger.info("POSITION_LIMIT：min=%s max=%s，内缩 %.4f m",
                    limits.lower.tolist(), limits.upper.tolist(), limits.margin)
        if plan is not None:
            logger.info("方式=%s，目标 TCP=%s，预计时间/预算=%.3f s",
                        args.motion_method, plan.target.tolist(), plan.duration)
            if args.return_home:
                logger.info("到位后仅让 %s 回预设 Home，回位阶段仍检查相同位置边界", args.arm)
        if args.preview:
            logger.info("PREVIEW_ONLY：未发送伺服、模式切换、Home 或移动命令")
            return
        control_started = True
        if not robot.operational():
            robot.ServoOn()  # 机器人级伺服操作，不改变手臂运动目标；不自动 ClearFault。
        deadline = time.monotonic() + args.startup_timeout
        while not robot.operational():
            checked_pose(robot, group, limits)
            if time.monotonic() >= deadline:
                raise TimeoutError("等待机器人伺服就绪超时")
            time.sleep(0.05)
        # 伺服就绪后重新取实际起点，避免使用启动前的旧位姿。
        start_pose = checked_pose(robot, group, limits)
        if args.home_only:
            home_with_limits(robot, group, sdk, args, limits, logger)
        else:
            plan = build_plan(start_pose, args, limits)
            if args.motion_method == "movel":
                movel_with_limits(robot, group, sdk, args, limits, plan, logger)
            else:
                move_with_limits(robot, group, sdk, args, limits, plan, logger)
            if args.return_home:
                # 只有正常到位才进入回位阶段；先停止，再执行同一手臂的 Home。
                # 不在 finally 中回位，避免中断、故障或越界时产生新的运动。
                robot.Stop()
                if robot.mode() != sdk.Mode.IDLE or not robot.all_stopped():
                    raise RuntimeError("斜向运动后未确认停止，取消回 Home")
                checked_pose(robot, group, limits)
                home_with_limits(robot, group, sdk, args, limits, logger)
    finally:
        if control_started:
            # Stop 为机器人级命令；正常结束、Ctrl+C、超时、越界、异常均执行。
            robot.Stop()
            if robot.mode() != sdk.Mode.IDLE or not robot.all_stopped():
                raise RuntimeError("Stop 已返回，但未确认 IDLE 及全部关节组停止")
            logger.info("STOP_CONFIRMED：IDLE，全部关节组已停止")


def main(argv=None):
    """argv：可选命令行参数列表；返回 0 成功、1 异常、130 用户中断。"""
    parser = build_parser()
    args = parser.parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")
    logger = logging.getLogger("SingleArmCartesian")
    try:
        run(args, logger=logger)
        return 0
    except KeyboardInterrupt:
        logger.info("用户中断，运动停止处理已执行")
        return 130
    except Exception as error:
        logger.error("%s", error)
        return 1


if __name__ == "__main__":
    sys.exit(main())
