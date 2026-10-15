"""Bounded RDK 2.1 read-only request/reply test; no robot setters or motion.

global_variables() blocks until a controller reply arrives. Only the number
of returned variables is printed; their names and values remain private.
"""
import argparse
import importlib.metadata
import os
from pathlib import Path
import subprocess
import sys
import time

from common import positive_seconds, validate_serial


def probe(serial):
    import flexivrdk

    robot = flexivrdk.Robot(serial)
    if not robot.connected():
        raise RuntimeError("Robot connection was not established")
    print("RDK_CONNECTED", flush=True)
    print("RDK license:", robot.info().license_type, flush=True)
    initial_mode = robot.mode()
    print("Control mode:", initial_mode, flush=True)
    for index in range(1, 4):
        start = time.perf_counter()
        reply = robot.global_variables()
        elapsed_ms = (time.perf_counter() - start) * 1000
        if not isinstance(reply, dict):
            raise RuntimeError("Unexpected controller reply type")
        if not robot.connected():
            raise RuntimeError("Connection lost during request/reply test")
        print(f"REQUEST_REPLY_OK {index}/3: {elapsed_ms:.3f} ms; "
              f"variable_count={len(reply)}", flush=True)
        time.sleep(0.2)
    if robot.mode() != initial_mode:
        raise RuntimeError("Control mode changed externally during the test")
    print("REQUEST_REPLY_VERIFIED: 3 read-only controller replies; "
          "motion command delivery remains untested", flush=True)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--serial", default=os.environ.get("FLEXIV_ROBOT_SERIAL"))
    parser.add_argument("--connect-readonly", action="store_true")
    parser.add_argument("--timeout-seconds", type=positive_seconds, default=20)
    parser.add_argument("--worker", action="store_true", help=argparse.SUPPRESS)
    args = parser.parse_args(argv)
    try:
        serial = validate_serial(args.serial)
        if not args.connect_readonly:
            raise ValueError("Read-only hardware opt-in required: --connect-readonly")
        if sys.prefix == sys.base_prefix or (hasattr(os, "geteuid") and os.geteuid() == 0):
            raise ValueError("Use the project venv without root privileges")
        if importlib.metadata.version("flexivrdk") != "2.1.0":
            raise ValueError("This test requires flexivrdk 2.1.0")
        if args.worker:
            probe(serial)
            return 0
        env = os.environ.copy()
        env["FLEXIV_ROBOT_SERIAL"] = serial
        command = [sys.executable, "-u", str(Path(__file__).resolve()),
                   "--connect-readonly", "--worker"]
        print("Testing 3 blocking read-only requests; timeout is failure.", flush=True)
        return subprocess.run(command, env=env, timeout=args.timeout_seconds).returncode
    except subprocess.TimeoutExpired:
        print("REQUEST_REPLY_FAILED: diagnostic timed out", file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        print("REQUEST_REPLY_CANCELLED", file=sys.stderr)
        return 130
    except Exception as exc:
        print("REQUEST_REPLY_FAILED:", str(exc), file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
