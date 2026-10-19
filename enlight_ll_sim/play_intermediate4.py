#!/usr/bin/env python3
"""Replay intermediate4's Cartesian reference path on the Enlight-LL URDF.

This process never imports flexivrdk or connects to a robot.  The original
example's build_plan()/MotionPlan.sample() supply the Cartesian motion;
kinematics.py converts each sampled flange pose to URDF joint positions.
"""

import argparse
import importlib.util
import math
import os
from pathlib import Path
import time

import numpy as np
from scipy.spatial.transform import Rotation

from kinematics import parse_chain


LAB_ROOT = Path(__file__).resolve().parents[2]
EXAMPLE = (
    LAB_ROOT
    / "rdk/flexiv_enlight_readonly_setup/third_party/flexiv_rdk/example_py"
    / "intermediate4_non_realtime_cartesian_motion_force_control.py"
)
XACRO = Path("/workspaces/src/flexiv_description/urdf/flexiv.urdf.xacro")


def load_example():
    spec = importlib.util.spec_from_file_location("intermediate4_motion", EXAMPLE)
    module = importlib.util.module_from_spec(spec)
    # dataclass resolves the module while the class body is being evaluated.
    import sys

    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def pose_from_matrix(transform):
    qx, qy, qz, qw = Rotation.from_matrix(transform[:3, :3]).as_quat()
    return np.array([*transform[:3, 3], qw, qx, qy, qz], dtype=float)


def matrix_from_pose(pose):
    transform = np.eye(4)
    transform[:3, 3] = pose[:3]
    transform[:3, :3] = Rotation.from_quat(
        [pose[4], pose[5], pose[6], pose[3]]
    ).as_matrix()
    return transform


def parse_args(argv=None):
    viewer = argparse.ArgumentParser(add_help=False)
    viewer.add_argument("--start-joints", type=float, nargs=7, metavar="RAD")
    viewer.add_argument("--once", action="store_true")
    viewer.add_argument("--check-only", action="store_true")
    viewer.add_argument("--frequency", type=int, default=30)
    own, motion_argv = viewer.parse_known_args(argv)

    motion = load_example()
    if not any(flag in motion_argv for flag in ("--delta", "--position", "--home-only")):
        motion_argv = ["--delta", "0.01", "0", "-0.01", "--tilt", "0", "15", "0", *motion_argv]
    serial = os.environ.get("OFFLINE_MODEL_SN", "Enlight-LL-123456")
    # The script's recorded safety box belongs to a specific physical setup.
    # The offline URDF origin is different, so use a broad virtual workspace.
    motion_argv = [
        "--position-min", "-2", "-2", "-2",
        "--position-max", "2", "2", "2",
        *motion_argv,
    ]
    args = motion.build_parser().parse_args([serial, str(own.frequency), *motion_argv])
    if args.home_only:
        raise ValueError("离线画面需要 --delta 或 --position；不回放 Home 原语")
    limits = motion.validate_args(args)
    return own, args, limits, motion


def make_joint_trajectory(chain, plan, initial_q, frequency):
    samples = max(2, math.ceil(plan.duration * frequency) + 1)
    times = np.linspace(0.0, plan.duration, samples)
    positions = [np.asarray(initial_q, dtype=float)]
    max_pos_error = 0.0
    max_angle_error = 0.0

    for elapsed in times[1:]:
        desired = matrix_from_pose(plan.sample(elapsed))
        q, pos_error, angle_error = chain.solve(desired, positions[-1])
        if pos_error > 0.002 or angle_error > math.radians(2):
            raise RuntimeError(
                f"轨迹在 t={elapsed:.2f}s 处无法匹配 URDF: "
                f"位置误差 {pos_error * 1000:.2f} mm, 姿态误差 {math.degrees(angle_error):.2f}°"
            )
        positions.append(q)
        max_pos_error = max(max_pos_error, pos_error)
        max_angle_error = max(max_angle_error, angle_error)

    return np.asarray(positions), max_pos_error, max_angle_error


def publish_trajectory(arm, left_chain, right_chain, trajectory, frequency, once):
    import rclpy
    from sensor_msgs.msg import JointState

    rclpy.init()
    node = rclpy.create_node("enlight_ll_offline_motion")
    publisher = node.create_publisher(JointState, "/joint_states", 10)
    other_q = right_chain.default_q if arm == "ARM_1" else left_chain.default_q
    names = list(left_chain.joint_names) + list(right_chain.joint_names)

    def publish(active_q):
        msg = JointState()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.name = names
        if arm == "ARM_1":
            msg.position = [*active_q.tolist(), *other_q.tolist()]
        else:
            msg.position = [*other_q.tolist(), *active_q.tolist()]
        publisher.publish(msg)

    def hold(active_q, duration):
        end = time.monotonic() + duration
        while rclpy.ok() and time.monotonic() < end:
            publish(active_q)
            time.sleep(1.0 / frequency)

    def play(frames):
        target_time = time.monotonic()
        for q in frames:
            if not rclpy.ok():
                break
            publish(q)
            target_time += 1.0 / frequency
            time.sleep(max(0.0, target_time - time.monotonic()))

    try:
        print("正在发布 /joint_states；RViz 中应看到 Enlight-LL 的单臂运动。Ctrl+C 退出。", flush=True)
        hold(trajectory[0], 3.0)
        while rclpy.ok():
            play(trajectory)
            hold(trajectory[-1], 1.0)
            if once:
                while rclpy.ok():
                    hold(trajectory[-1], 1.0)
                break
            play(trajectory[::-1])
            hold(trajectory[0], 1.0)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main(argv=None):
    own, args, limits, motion = parse_args(argv)

    import xacro

    urdf_xml = xacro.process_file(
        str(XACRO), mappings={"robot_type": "Enlight-LL", "robot_sn": args.robot_sn}
    ).toxml()
    left_chain = parse_chain(urdf_xml, "ARM_1", args.robot_sn)
    right_chain = parse_chain(urdf_xml, "ARM_2", args.robot_sn)
    chain = left_chain if args.arm == "ARM_1" else right_chain
    initial_q = np.asarray(own.start_joints if own.start_joints else chain.default_q, dtype=float)
    start_pose = pose_from_matrix(chain.forward(initial_q))
    plan = motion.build_plan(start_pose, args, limits)
    trajectory, pos_error, angle_error = make_joint_trajectory(
        chain, plan, initial_q, own.frequency
    )

    print(f"模型 Enlight-LL | {args.arm} | {len(trajectory)} 帧 | {plan.duration:.2f} s", flush=True)
    print(f"虚拟世界系起点 TCP=flange: {np.round(plan.start, 5).tolist()}", flush=True)
    print(f"虚拟世界系终点 TCP=flange: {np.round(plan.target, 5).tolist()}", flush=True)
    print(
        f"URDF 正逆解最大误差: {pos_error * 1000:.3f} mm, "
        f"{math.degrees(angle_error):.3f}°",
        flush=True,
    )
    if own.check_only:
        return
    publish_trajectory(args.arm, left_chain, right_chain, trajectory, own.frequency, own.once)


if __name__ == "__main__":
    main()
