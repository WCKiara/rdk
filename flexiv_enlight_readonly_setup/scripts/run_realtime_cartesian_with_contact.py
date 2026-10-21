"""Supervise the current ARM_1 motion example and whole-arm contact CSV recording.

Run this launcher with sudo and the project's Python interpreter. The motion
child inherits the scheduler privileges; the read-only recorder runs as the
original sudo user so its CSV remains user-owned. Motion options such as
--polish, --tilt-method and --tilt-only are forwarded to the motion example.
--preview is for older binaries that support read-only preview; select those
explicitly with --rt-binary.
"""

import argparse
import csv
from datetime import datetime, timezone
import math
import os
from pathlib import Path
import pwd
import signal
import subprocess
import sys
import time


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_BINARY = ROOT / "third_party/flexiv_rdk/example/build/intermediate6_realtime_cartesian_motion_force_control"
RECORDER = ROOT / "scripts/record_whole_arm_contact.py"
POLL_PERIOD = 0.05
STOP_SIGNALS = (signal.SIGINT, signal.SIGTERM, signal.SIGHUP)


class StopRequested(Exception):
    def __init__(self, signum):
        self.signum = signum
        super().__init__(f"signal {signum}")


class StopState:
    """Signal handlers only set state, including during child creation/cleanup."""

    def __init__(self):
        self.signum = None

    def request(self, signum, _frame=None):
        if self.signum is None:
            self.signum = signum

    def check(self):
        if self.signum is not None:
            raise StopRequested(self.signum)


def positive_float(value):
    result = float(value)
    if not math.isfinite(result) or result <= 0:
        raise argparse.ArgumentTypeError("must be finite and positive")
    return result


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("robot_sn")
    parser.add_argument("--arm", choices=("ARM_1",), default="ARM_1",
                        help="This launcher permits ARM_1 only")
    parser.add_argument("--record-contact", action="store_true", default=True,
                        help="Record ARM_1 whole-arm contact estimates (enabled by default)")
    parser.add_argument("--contact-rate", type=positive_float, default=20.0)
    parser.add_argument("--contact-output", type=Path,
                        help="New CSV path; default: logs/whole_arm_contact_arm1_<UTC timestamp>.csv")
    parser.add_argument("--contact-ready-timeout", type=positive_float, default=15.0)
    parser.add_argument("--rt-binary", type=Path, default=DEFAULT_BINARY)
    parser.add_argument("--preview", action="store_true",
                        help="Legacy binaries only: run read-only preview without recording; use --rt-binary")
    args, forwarded = parser.parse_known_args(argv)
    if "--" in forwarded:
        parser.error("The '--' separator is unsupported; pass motion options directly")
    args.rt_args = forwarded
    if args.preview and args.rt_binary == DEFAULT_BINARY:
        parser.error("The current motion example has no --preview; select a preview-capable legacy --rt-binary")
    return args


def motion_command(args):
    command = [str(args.rt_binary), args.robot_sn, *args.rt_args, "--arm", "ARM_1"]
    if args.preview:
        command.append("--preview")
    return command


def contact_output(args):
    if args.contact_output is not None:
        return args.contact_output.absolute()
    timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S")
    return ROOT / "logs" / f"whole_arm_contact_arm1_{timestamp}_{time.time_ns() % 1000000000:09d}Z.csv"


def recorder_command(args, output):
    return [sys.executable, "-u", str(RECORDER), args.robot_sn,
            "--arm", "ARM_1", "--rate", str(args.contact_rate),
            "--duration", "86400", "--output", str(output)]


def recorder_identity():
    """Use subprocess's native privilege drop, avoiding a Python preexec_fn."""
    if os.geteuid() != 0 or "SUDO_UID" not in os.environ:
        return {}
    uid = int(os.environ["SUDO_UID"])
    original_user = pwd.getpwuid(uid)
    return {"user": uid, "group": original_user.pw_gid,
            "extra_groups": os.getgrouplist(original_user.pw_name, original_user.pw_gid)}


def has_contact_row(output):
    try:
        with output.open(newline="", encoding="utf-8") as stream:
            row = next(csv.DictReader(stream), None)
        if row is None or row.get("joint_group") != "ARM_1":
            return False
        if int(row["sample"]) < 0 or int(row["link_index"]) < 0:
            return False
        keys = ("force_x_N", "force_y_N", "force_z_N",
                "position_x_m", "position_y_m", "position_z_m")
        return all(math.isfinite(float(row[key])) for key in keys)
    except (FileNotFoundError, KeyError, TypeError, ValueError, csv.Error):
        return False


def wait_for_contact(recorder, output, timeout, stop):
    deadline = time.monotonic() + timeout
    while True:
        stop.check()
        code = recorder.poll()
        if code is not None:
            raise RuntimeError(f"Contact recorder exited before motion (status {code}); motion was not started")
        if has_contact_row(output):
            return
        if time.monotonic() >= deadline:
            raise RuntimeError("Contact recorder did not produce a valid ARM_1 data row; motion was not started")
        time.sleep(POLL_PERIOD)


def stop_process(child, first_signal=signal.SIGINT):
    if child is None or child.poll() is not None:
        return
    for signum, timeout in ((first_signal, 5), (signal.SIGTERM, 3), (signal.SIGKILL, 3)):
        try:
            os.killpg(child.pid, signum)
        except ProcessLookupError:
            pass
        try:
            child.wait(timeout=timeout)
            return
        except subprocess.TimeoutExpired:
            continue
    raise RuntimeError(f"Unable to reap child process {child.pid}")


def supervise(args, stop):
    recorder = None
    motion = None
    try:
        stop.check()
        if not args.preview:
            output = contact_output(args)
            if output.exists() or output.is_symlink():
                raise ValueError(f"Contact CSV already exists; choose a new --contact-output: {output}")
            recorder = subprocess.Popen(recorder_command(args, output), cwd=ROOT,
                                        start_new_session=True, **recorder_identity())
            wait_for_contact(recorder, output, args.contact_ready_timeout, stop)
            print(f"CONTACT_RECORDING_READY: {output}", flush=True)
        stop.check()
        motion = subprocess.Popen(motion_command(args), cwd=ROOT, start_new_session=True)
        while True:
            stop.check()
            if recorder is not None:
                recorder_code = recorder.poll()
                if recorder_code is not None:
                    raise RuntimeError(f"Contact recorder exited during motion (status {recorder_code}); stopping motion")
            motion_code = motion.poll()
            if motion_code is not None:
                return motion_code
            time.sleep(POLL_PERIOD)
    finally:
        try:
            stop_process(motion)
        finally:
            # CSV is flushed by the recorder after every sample; SIGTERM avoids
            # a normal completion printing an unnecessary KeyboardInterrupt traceback.
            stop_process(recorder, signal.SIGTERM)


def main(argv=None):
    args = parse_args(argv)
    stop = StopState()
    old_handlers = {}
    try:
        args.rt_binary = args.rt_binary.absolute()
        if not args.rt_binary.is_file() or not os.access(args.rt_binary, os.X_OK):
            raise ValueError(f"Real-time binary is missing or not executable: {args.rt_binary}")
        if not args.preview and os.geteuid() != 0:
            raise ValueError("Run this launcher with sudo and the project's .venv/bin/python for real-time scheduler privileges")
        for signum in STOP_SIGNALS:
            old_handlers[signum] = signal.signal(signum, stop.request)
        return supervise(args, stop)
    except StopRequested as exc:
        print("Motion and contact recording stopped.", file=sys.stderr)
        return 128 + exc.signum
    except KeyboardInterrupt:
        return 130
    except (ValueError, RuntimeError, OSError, subprocess.SubprocessError) as exc:
        print(f"REALTIME_CONTACT_FAILED: {exc}", file=sys.stderr)
        return 2
    finally:
        for signum, handler in old_handlers.items():
            signal.signal(signum, handler)


if __name__ == "__main__":
    sys.exit(main())
