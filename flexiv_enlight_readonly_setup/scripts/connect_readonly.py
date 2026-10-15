"""RDK 2.1 connection/telemetry diagnostic for the operator's actual controller.

Review before approving --connect-readonly. Only Robot construction and the
connected/info/states accessors are used. No actuation, fault clearing, enable,
mode changes, or tool commands. The outer process bounds initialization time.
"""
import argparse
import importlib.metadata
import math
import os
import subprocess
import sys
import time
from common import positive_seconds, validate_serial
from run_official_state_example import run_child


def observe(serial):
    import flexivrdk
    robot = flexivrdk.Robot(serial)
    if not robot.connected():
        raise RuntimeError('RDK initialization returned without a connected robot')
    info = robot.info()
    print('RDK_CONNECTED', flush=True)
    print('Model:', flexivrdk.kProductModelNames[info.product_model], flush=True)
    print('Serial:', info.serial_num, flush=True)
    print('Robot software:', info.software_ver, flush=True)
    print('RDK license:', info.license_type, flush=True)
    if not info.single_arm_groups:
        raise RuntimeError('Controller has no single-arm state groups')
    previous = {}
    advancing = set()
    for _ in range(3):
        groups = robot.states()
        for group, name in info.single_arm_groups.items():
            if group not in groups:
                raise RuntimeError('Missing arm state: ' + name)
            state = groups[group]
            q = list(state.q)
            if not q or len(q) != info.DoF[group] or not all(math.isfinite(x) for x in q):
                raise RuntimeError('Invalid joint sample: ' + name)
            stamp = tuple(state.timestamp)
            if group in previous and stamp > previous[group]:
                advancing.add(group)
            previous[group] = stamp
            print(f'{name}: timestamp={stamp} q={q} dq={list(state.dq)}', flush=True)
        time.sleep(1)
    if advancing != set(info.single_arm_groups):
        raise RuntimeError('Connection succeeded, but advancing state timestamps were not verified for every arm')
    print('LIVE_STATES_VERIFIED: finite joint values and advancing timestamps', flush=True)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--serial', default=os.environ.get('FLEXIV_ROBOT_SERIAL'))
    parser.add_argument('--connect-readonly', action='store_true')
    parser.add_argument('--timeout-seconds', type=positive_seconds, default=15)
    parser.add_argument('--worker', action='store_true', help=argparse.SUPPRESS)
    args = parser.parse_args(argv)
    try:
        serial = validate_serial(args.serial)
        if not args.connect_readonly:
            raise ValueError('Operator approval after source inspection is required: --connect-readonly')
        if sys.prefix == sys.base_prefix or (hasattr(os, 'geteuid') and os.geteuid() == 0):
            raise ValueError('Use the project venv without root privileges')
        if importlib.metadata.version('flexivrdk') != '2.1.0':
            raise ValueError('This reviewed diagnostic requires flexivrdk 2.1.0')
        if args.worker:
            observe(serial)
            return 0
        command = [sys.executable, '-u', os.path.abspath(__file__), '--serial', serial,
                   '--connect-readonly', '--worker']
        print('Read-only command:', subprocess.list2cmdline(command), flush=True)
        return run_child(command, os.path.dirname(__file__), args.timeout_seconds)
    except KeyboardInterrupt:
        return 0
    except Exception as exc:
        print('RDK_CONNECTION_FAILED:', str(exc), file=sys.stderr)
        return 2


if __name__ == '__main__':
    sys.exit(main())
