#!/usr/bin/env python3
"""Export an Enlight-LL URDF and the intermediate4 reference motion for Isaac Sim.

This is an offline preparation step.  It does not import Isaac Sim or the
Flexiv SDK, and it never contacts a robot.
"""

import argparse
import json
import math
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

import numpy as np
import xacro

from kinematics import parse_chain
from play_intermediate4 import (
    XACRO,
    make_joint_trajectory,
    parse_args,
    pose_from_matrix,
)


DESCRIPTION_ROOT = Path("/workspaces/src/flexiv_description")
DEFAULT_OUTPUT = Path("/lab/rdk/enlight_ll_sim/isaac_bundle")
MESH_PREFIX = "package://flexiv_description/"


def _copy_obj_materials(obj_source, obj_relative, output):
    """Keep OBJ material files beside the mesh when the OBJ names them."""
    for line in obj_source.read_text(errors="replace").splitlines():
        if not line.startswith("mtllib "):
            continue
        for filename in line.split()[1:]:
            relative = obj_relative.parent / filename
            source = DESCRIPTION_ROOT / relative
            if not source.exists():
                # The upstream OBJs name e.g. Link0.mtl, while the checked-in
                # file is link0.mtl.  Keep the OBJ's spelling in the bundle.
                source = next(
                    path for path in source.parent.iterdir()
                    if path.name.casefold() == filename.casefold()
                )
            target = output / relative
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(source, target)


def save_portable_urdf(urdf_xml, output):
    """Turn package mesh references into paths local to the exported URDF."""
    root = ET.fromstring(urdf_xml)
    copied = set()
    for mesh in root.iter("mesh"):
        uri = mesh.get("filename")
        if not uri or not uri.startswith(MESH_PREFIX):
            continue
        relative = Path(uri[len(MESH_PREFIX):])
        mesh.set("filename", relative.as_posix())
        if relative in copied:
            continue
        source = DESCRIPTION_ROOT / relative
        target = output / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, target)
        if source.suffix.lower() == ".obj":
            _copy_obj_materials(source, relative, output)
        copied.add(relative)

    urdf_file = output / "robot.urdf"
    ET.ElementTree(root).write(urdf_file, encoding="utf-8", xml_declaration=True)
    return urdf_file, len(copied)


def main(argv=None):
    exporter = argparse.ArgumentParser(add_help=False)
    exporter.add_argument("--out-dir", type=Path, default=DEFAULT_OUTPUT)
    export_options, motion_argv = exporter.parse_known_args(argv)
    own, args, limits, motion = parse_args(motion_argv)

    urdf_xml = xacro.process_file(
        str(XACRO), mappings={"robot_type": "Enlight-LL", "robot_sn": args.robot_sn}
    ).toxml()
    left_chain = parse_chain(urdf_xml, "ARM_1", args.robot_sn)
    right_chain = parse_chain(urdf_xml, "ARM_2", args.robot_sn)
    active_chain = left_chain if args.arm == "ARM_1" else right_chain
    initial_q = np.asarray(
        own.start_joints if own.start_joints else active_chain.default_q,
        dtype=float,
    )
    start_pose = pose_from_matrix(active_chain.forward(initial_q))
    plan = motion.build_plan(start_pose, args, limits)
    trajectory, pos_error, angle_error = make_joint_trajectory(
        active_chain, plan, initial_q, own.frequency
    )

    if args.arm == "ARM_1":
        other_q = np.broadcast_to(right_chain.default_q, trajectory.shape)
        positions = np.concatenate((trajectory, other_q), axis=1)
        side = "left"
    else:
        other_q = np.broadcast_to(left_chain.default_q, trajectory.shape)
        positions = np.concatenate((other_q, trajectory), axis=1)
        side = "right"

    sample_times = np.linspace(0.0, plan.duration, len(trajectory))
    output = export_options.out_dir.resolve()
    output.mkdir(parents=True, exist_ok=True)
    urdf_file, mesh_count = save_portable_urdf(urdf_xml, output)

    payload = {
        "schema": "enlight_ll_isaac_motion_v1",
        "robot_type": "Enlight-LL",
        "robot_sn": args.robot_sn,
        "urdf_file": urdf_file.name,
        "arm": args.arm,
        "world_frame": "world",
        "tcp_frame": f"{side}_{args.robot_sn}_flange",
        "tcp_assumption": "flange_is_tcp",
        "pose_format": "[x_m, y_m, z_m, qw, qx, qy, qz]",
        "frequency_hz": own.frequency,
        "dt_s": float(plan.duration / (len(trajectory) - 1)),
        "sample_times_s": sample_times.tolist(),
        "joint_names": [*left_chain.joint_names, *right_chain.joint_names],
        "positions_rad": positions.tolist(),
        "start_pose": plan.start.tolist(),
        "target_pose": plan.target.tolist(),
        "max_position_error_m": pos_error,
        "max_orientation_error_rad": angle_error,
        "source_script": (
            "rdk/flexiv_enlight_readonly_setup/third_party/flexiv_rdk/example_py/"
            "intermediate4_non_realtime_cartesian_motion_force_control.py"
        ),
        "reference_motion": "MotionPlan.sample; controller execution may differ",
    }
    motion_file = output / "motion.json"
    motion_file.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n")

    print(f"Isaac bundle: {output}")
    print(f"  URDF: {urdf_file.name}; {mesh_count} referenced meshes")
    print(f"  Motion: {motion_file.name}; {len(trajectory)} frames over {plan.duration:.3f} s")
    print(f"  Arm: {args.arm}; max IK error {pos_error * 1000:.3f} mm, "
          f"{math.degrees(angle_error):.3f} deg")


if __name__ == "__main__":
    main()
