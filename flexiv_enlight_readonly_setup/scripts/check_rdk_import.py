"""Import in a timed subprocess, without constructing a robot."""
import argparse
import sys
from common import positive_seconds, run_command


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--timeout', type=positive_seconds, default=15)
    args = parser.parse_args()
    code = "import flexivrdk; import importlib.metadata as m; print('RDK import OK'); print('flexivrdk version:', m.version('flexivrdk'))"
    result = run_command([sys.executable, '-c', code], args.timeout)
    print(result.get('stdout', ''))
    if result.get('stderr'):
        print(result['stderr'], file=sys.stderr)
    if result.get('error'):
        print('FAIL:', result['error'], file=sys.stderr)
    return 0 if result['code'] == 0 else 1


if __name__ == '__main__':
    sys.exit(main())
