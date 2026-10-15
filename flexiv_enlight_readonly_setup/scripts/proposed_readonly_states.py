"""PROPOSAL FOR REVIEW: not the official example. Requires separate operator approval.

Only connects and reads info/states, using the reviewed upstream accessor APIs.
No fault clearing, servo commands, control-mode changes, or motion commands.
The outer process imposes a timeout even if the Robot constructor blocks.
"""
import argparse
import math
import os
import subprocess
import sys
import time
from common import positive_seconds, validate_serial
from run_official_state_example import run_child


def read_states(serial):
    import flexivrdk
    robot = flexivrdk.Robot(serial)
    info = robot.info()
    if len(info.single_arm_groups) != 1:
        raise RuntimeError('This project requires exactly one arm')
    previous = {}
    observed_live = False
    while True:
        groups = robot.states()
        for group in info.single_arm_groups:
            if group not in groups:
                raise RuntimeError('No arm state received')
            state = groups[group]
            q = list(state.q)
            if not q or len(q) != info.DoF[group] or not all(math.isfinite(value) for value in q):
                raise RuntimeError('Invalid/empty joint values')
            stamp = tuple(state.timestamp)
            if group in previous and stamp > previous[group] and not observed_live:
                print('LIVE_SAMPLE: finite joint values with advancing timestamp', flush=True)
                observed_live = True
            previous[group] = stamp
            print(f'timestamp={stamp} q={q} dq={list(state.dq)} tcp_pose={list(state.tcp_pose)}', flush=True)
        time.sleep(1)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--serial', default=os.environ.get('FLEXIV_ROBOT_SERIAL'))
    parser.add_argument('--connect-readonly', action='store_true', help='Requires approval of THIS alternative after inspection')
    parser.add_argument('--timeout-seconds', type=positive_seconds, default=15)
    parser.add_argument('--worker', action='store_true', help=argparse.SUPPRESS)
    args = parser.parse_args()
    try:
        serial = validate_serial(args.serial)
        if not args.connect_readonly:
            raise ValueError('Review this alternative and obtain explicit operator approval before --connect-readonly')
        if sys.prefix == sys.base_prefix or (hasattr(os, 'geteuid') and os.geteuid() == 0):
            raise ValueError('Use the project venv without root privilege')
        if args.worker:
            read_states(serial)
            return 0
        command = [sys.executable, '-u', os.path.abspath(__file__), '--worker', '--serial', serial, '--connect-readonly']
        print('Proposed read-only diagnostic (local serial):', subprocess.list2cmdline(command), flush=True)
        return run_child(command, os.path.dirname(__file__), args.timeout_seconds)
    except KeyboardInterrupt:
        return 0
    except Exception as exc:
        print('REFUSED/FAILED:', str(exc), file=sys.stderr)
        return 2


if __name__ == '__main__':
    sys.exit(main())
