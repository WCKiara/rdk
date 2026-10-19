"""Read and record per-link multi-contact estimates without commanding the robot."""

import argparse
import csv
import time
from pathlib import Path

def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("robot_sn", help="Robot serial number, e.g. Enlight-L-123456")
    parser.add_argument("--output", type=Path, default=Path("logs/whole_arm_contact.csv"))
    parser.add_argument("--rate", type=float, default=20.0, help="Sampling rate in Hz")
    parser.add_argument("--duration", type=float, default=30.0, help="Recording length in seconds")
    parser.add_argument("--show-live", action="store_true", help="Print each link estimate while recording")
    args = parser.parse_args(argv)
    if args.rate <= 0 or args.duration <= 0:
        parser.error("--rate and --duration must be positive")

    import flexivrdk

    if not all(hasattr(flexivrdk.Model, method) for method in
               ("multi_contact_forces", "multi_contact_positions")):
        raise RuntimeError("This flexivrdk Python package does not expose multi-contact Model methods")

    robot = flexivrdk.Robot(args.robot_sn)
    model = flexivrdk.Model(robot)
    period = 1.0 / args.rate
    stop_at = time.monotonic() + args.duration
    sample = 0
    rows = 0

    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow([
            "sample", "host_time_ns", "robot_time_sec", "robot_time_nsec",
            "joint_group", "link_index", "force_x_N", "force_y_N", "force_z_N",
            "position_x_m", "position_y_m", "position_z_m",
        ])
        while time.monotonic() < stop_at:
            started = time.monotonic()
            states = robot.states()
            all_states = states[flexivrdk.JointGroup.ALL]
            model.Update(all_states.q, all_states.dq)
            forces = model.multi_contact_forces()
            positions = model.multi_contact_positions()
            host_time_ns = time.time_ns()
            sec, nsec = all_states.timestamp

            for group, group_forces in forces.items():
                group_positions = positions[group]
                if len(group_forces) != len(group_positions):
                    raise RuntimeError("Multi-contact force and position counts differ")
                for link_index, (force, position) in enumerate(zip(group_forces, group_positions)):
                    group_name = flexivrdk.kJointGroupNames[group]
                    writer.writerow([
                        sample, host_time_ns, sec, nsec,
                        group_name, link_index,
                        force[0], force[1], force[2],
                        position[0], position[1], position[2],
                    ])
                    rows += 1
                    if args.show_live:
                        print(f"{group_name} link {link_index}: force={list(force)} "
                              f"position={list(position)}", flush=True)

            sample += 1
            stream.flush()
            time.sleep(max(0.0, period - (time.monotonic() - started)))

    print(f"Recorded {sample} samples and {rows} link rows to {args.output}")
    if rows == 0:
        print("No multi-contact data returned. This robot model may not support the feature.")


if __name__ == "__main__":
    main()
