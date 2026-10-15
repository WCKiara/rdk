"""Fail-closed wrapper around the single reviewed, unchanged upstream example."""
import argparse
import ast
import hashlib
import json
import os
import signal
import subprocess
import sys
from pathlib import Path
from common import positive_seconds, validate_serial

ROOT = Path(__file__).resolve().parents[1]
RELATIVE = Path('third_party/flexiv_rdk/example_py/basics1_display_robot_states.py')
REVIEW = ROOT / 'docs/source_review.json'
READ_METHODS = {'info', 'states', 'actions', 'digital_inputs', 'digital_outputs', 'fault', 'operational'}


def verify_source(root=ROOT):
    source = root / RELATIVE
    if not source.is_file() or source.is_symlink():
        raise ValueError('Missing official example or symlink: clone the official repository')
    review_path = root / 'docs/source_review.json'
    if not review_path.is_file():
        raise ValueError('No manual source inspection record; inspect before connecting')
    review = json.loads(review_path.read_text(encoding='utf-8'))
    digest = hashlib.sha256(source.read_bytes()).hexdigest()
    if review.get('path') != RELATIVE.as_posix() or review.get('sha256') != digest:
        raise ValueError('Source changed after inspection; re-inspection required')
    if review.get('readonly_approved') is not True:
        raise ValueError('Upstream example is NOT read-only: ' + review.get('reason', 'not approved'))
    tree = ast.parse(source.read_text(encoding='utf-8'))
    for call in ast.walk(tree):
        if isinstance(call, ast.Call) and isinstance(call.func, ast.Attribute):
            owner = call.func.value
            if isinstance(owner, ast.Name) and owner.id == 'robot' and call.func.attr not in READ_METHODS:
                raise ValueError('Unapproved robot API call: ' + call.func.attr)
    result = subprocess.run(['git', '-C', str(root / 'third_party/flexiv_rdk'), 'rev-parse', 'HEAD'],
                            capture_output=True, text=True, timeout=10, check=True)
    if result.stdout.strip() != review.get('commit'):
        raise ValueError('Upstream commit changed; re-inspection required')
    return source


def build_command(serial, root=ROOT):
    return [sys.executable, '-u', str(verify_source(root)), validate_serial(serial)]


def run_child(command, cwd, timeout):
    child = subprocess.Popen(command, cwd=cwd)
    try:
        return child.wait(timeout=timeout)
    except (subprocess.TimeoutExpired, KeyboardInterrupt):
        print('\nStopping the state display process; this does not issue a robot command.', flush=True)
        if os.name != 'nt':
            child.send_signal(signal.SIGINT)
        else:
            child.terminate()
        try:
            child.wait(timeout=5)
        except subprocess.TimeoutExpired:
            child.terminate()
            try:
                child.wait(timeout=3)
            except subprocess.TimeoutExpired:
                child.kill()
                child.wait()
        print('Display ended; verify actual state output before declaring success.')
        return 0


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--serial', default=os.environ.get('FLEXIV_ROBOT_SERIAL'))
    parser.add_argument('--connect-readonly', action='store_true', help='Explicit operator approval for read-only hardware contact')
    parser.add_argument('--timeout-seconds', type=positive_seconds, default=15)
    args = parser.parse_args(argv)
    try:
        validate_serial(args.serial)
        if not args.connect_readonly:
            raise ValueError('Hardware access refused: --connect-readonly requires operator approval')
        if sys.prefix == sys.base_prefix:
            raise ValueError('Activate the project venv before running hardware diagnostics')
        if hasattr(os, 'geteuid') and os.geteuid() == 0:
            raise ValueError('Never run this hardware example as root')
        command = build_command(args.serial)
        print('Invocation (contains local serial):', subprocess.list2cmdline(command), flush=True)
        print('Read-only display only. Ctrl+C stops the display.', flush=True)
        return run_child(command, Path(command[2]).parent, args.timeout_seconds)
    except (ValueError, OSError, subprocess.SubprocessError) as exc:
        print('REFUSED:', str(exc), file=sys.stderr)
        return 2


if __name__ == '__main__':
    sys.exit(main())
