#!/usr/bin/env python3
"""Show the offline intermediate4 Enlight-LL motion in Isaac Sim.

The motion and URDF are prepared by prepare_isaac_bundle.sh. This script
does not import flexivrdk or contact a physical robot. --check-bundle uses
only Python's standard library and does not start Isaac Sim.
"""

import argparse
import json
import math
from pathlib import Path
import re
import time
import xml.etree.ElementTree as ET


DEFAULT_BUNDLE = Path(__file__).resolve().parent / "isaac_bundle"
ROBOT_PRIM = "/World/EnlightLL"


def load_bundle(bundle):
    """Check the inputs before starting the expensive simulator process."""
    motion_file = bundle / "motion.json"
    motion = json.loads(motion_file.read_text(encoding="utf-8"))
    if motion.get("schema") != "enlight_ll_isaac_motion_v1":
        raise ValueError(f"Unsupported motion schema in {motion_file}")

    urdf_file = bundle / motion["urdf_file"]
    robot = ET.parse(urdf_file).getroot()
    joint_names = motion["joint_names"]
    urdf_joints = {joint.get("name") for joint in robot.iter("joint")
                   if joint.get("type") != "fixed"}
    missing_joints = set(joint_names) - urdf_joints
    if missing_joints:
        raise ValueError(f"Motion joints absent from URDF: {sorted(missing_joints)}")

    times = motion["sample_times_s"]
    positions = motion["positions_rad"]
    if not times or len(times) != len(positions):
        raise ValueError("sample_times_s and positions_rad must have the same nonzero length")
    if len(set(joint_names)) != len(joint_names):
        raise ValueError("joint_names contains duplicate names")
    if any(len(row) != len(joint_names) for row in positions):
        raise ValueError("Each position row must match joint_names")
    if times[0] != 0 or any(b <= a for a, b in zip(times, times[1:])):
        raise ValueError("sample_times_s must increase from zero")
    if not all(math.isfinite(value) for row in positions for value in row):
        raise ValueError("positions_rad contains a nonfinite value")

    mesh_refs = [mesh.get("filename") for mesh in robot.iter("mesh")]
    missing_meshes = [name for name in mesh_refs if name and not (bundle / name).is_file()]
    if missing_meshes:
        raise FileNotFoundError(f"URDF meshes missing: {missing_meshes}")
    return motion, urdf_file, len(mesh_refs)


def import_robot(urdf_file, output_dir):
    """Use the importer available in Isaac Sim 6.x or 5.1."""
    try:
        from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig
    except ImportError:
        import omni.kit.commands

        output_path = output_dir / "enlight_ll.usd"
        ok, config = omni.kit.commands.execute("URDFCreateImportConfig")
        if not ok:
            raise RuntimeError("Isaac Sim could not create a URDF import configuration")
        config.merge_fixed_joints = False
        config.collision_from_visuals = False
        config.fix_base = True
        config.make_default_prim = True
        config.distance_scale = 1.0
        ok, _ = omni.kit.commands.execute(
            "URDFParseAndImportFile",
            urdf_path=urdf_file.as_posix(),
            import_config=config,
            dest_path=output_path.as_posix(),
        )
        if not ok:
            raise RuntimeError(f"Isaac Sim could not import {urdf_file}")
        return output_path

    # In Isaac Sim 6.x usd_path is an output directory; import_urdf()
    # returns the path to the generated USD file.
    config = URDFImporterConfig(
        urdf_path=urdf_file.as_posix(),
        usd_path=output_dir.as_posix(),
        merge_fixed_joints=False,
        merge_mesh=False,
        collision_from_visuals=False,
        fix_base=True,
    )
    return Path(URDFImporter(config).import_urdf())


def joint_indices(source_names, simulator_names):
    """Map URDF names to Isaac DOFs even if USD sanitized hyphens."""
    def usd_key(name):
        return re.sub(r"[^A-Za-z0-9_]", "_", name)

    by_name = {usd_key(name): index for index, name in enumerate(simulator_names)}
    absent = [name for name in source_names if usd_key(name) not in by_name]
    if absent:
        raise RuntimeError(
            f"Isaac articulation is missing these URDF joints: {absent}. "
            f"Available DOFs: {list(simulator_names)}"
        )
    return [by_name[usd_key(name)] for name in source_names]


def run_isaac(bundle, motion, urdf_file, once):
    # SimulationApp must exist before importing Omni/Isaac modules.
    from isaacsim import SimulationApp

    simulation_app = SimulationApp({"headless": False})
    try:
        import numpy as np
        import omni.kit.app
        import omni.usd

        manager = omni.kit.app.get_app().get_extension_manager()
        manager.set_extension_enabled_immediate("isaacsim.asset.importer.urdf", True)
        manager.set_extension_enabled_immediate("isaacsim.core.api", True)
        simulation_app.update()

        from pxr import UsdGeom
        from isaacsim.core.api import World
        from isaacsim.core.api.robots import Robot
        from isaacsim.core.utils.prims import get_articulation_root_api_prim_path
        from isaacsim.core.utils.viewports import set_camera_view

        output_dir = bundle / "isaac_import"
        output_dir.mkdir(exist_ok=True)
        usd_file = import_robot(urdf_file, output_dir)
        print(f"Imported URDF to {usd_file}", flush=True)

        dt = float(motion["dt_s"])
        world = World(stage_units_in_meters=1.0, physics_dt=dt, rendering_dt=dt)
        stage = omni.usd.get_context().get_stage()
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)
        stage.DefinePrim(ROBOT_PRIM, "Xform").GetReferences().AddReference(usd_file.as_posix())
        articulation_prim = get_articulation_root_api_prim_path(ROBOT_PRIM)
        robot = world.scene.add(Robot(prim_path=articulation_prim, name="enlight_ll"))
        world.scene.add_ground_plane(size=4.0, z_position=-0.05)
        set_camera_view(eye=np.array([1.6, -1.8, 1.35]),
                        target=np.array([0.0, 0.0, 0.45]))
        world.reset()

        indices = np.asarray(joint_indices(motion["joint_names"], robot.dof_names), dtype=np.int32)
        frames = np.asarray(motion["positions_rad"], dtype=np.float64)
        times = np.asarray(motion["sample_times_s"], dtype=np.float64)
        print(f"Articulation: {articulation_prim}; mapped {len(indices)} joints", flush=True)
        print(f"World: Z-up, meters, URDF root at origin; TCP: {motion['tcp_frame']}", flush=True)

        def show_pose(q):
            # This is a reference-path viewer: render the teleported joint
            # state without advancing PhysX, so gravity/drives cannot change
            # the commanded pose before it reaches the viewport.
            robot.set_joint_positions(q, joint_indices=indices)
            world.render()

        def hold_pose(q, seconds):
            end = time.monotonic() + seconds
            while simulation_app.is_running() and time.monotonic() < end:
                show_pose(q)
                time.sleep(dt)

        def play(sequence, sample_times):
            started = time.monotonic()
            for q, elapsed in zip(sequence, sample_times):
                if not simulation_app.is_running():
                    break
                time.sleep(max(0.0, started + float(elapsed) - time.monotonic()))
                show_pose(q)

        hold_pose(frames[0], 1.0)
        while simulation_app.is_running():
            play(frames, times)
            if once:
                while simulation_app.is_running():
                    show_pose(frames[-1])
                    time.sleep(dt)
                break
            hold_pose(frames[-1], 0.5)
            play(frames[::-1], times[-1] - times[::-1])
            hold_pose(frames[0], 0.5)
    finally:
        simulation_app.close()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bundle", type=Path, default=DEFAULT_BUNDLE)
    parser.add_argument("--check-bundle", action="store_true",
                        help="validate URDF, meshes and motion without launching Isaac Sim")
    parser.add_argument("--once", action="store_true",
                        help="play the source motion once, then hold the final pose")
    args = parser.parse_args(argv)

    bundle = args.bundle.resolve()
    motion, urdf_file, mesh_count = load_bundle(bundle)
    print(f"Bundle OK: {motion['robot_type']} {motion['arm']}; "
          f"{len(motion['positions_rad'])} frames, {len(motion['joint_names'])} joints, "
          f"{motion['sample_times_s'][-1]:.3f} s, {mesh_count} mesh references", flush=True)
    if not args.check_bundle:
        run_isaac(bundle, motion, urdf_file, args.once)


if __name__ == "__main__":
    main()
