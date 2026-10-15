"""Passive environment inspection; no robot import or connection."""
import json
import platform
import shutil
import struct
import sys
from pathlib import Path
from common import run_command

# Verified against official README at commit recorded in docs/source_review.json.
SUPPORTED = {"Linux": {(3, 10), (3, 12), (3, 14)},
             "Windows": {(3, 10), (3, 12)}, "Darwin": {(3, 10), (3, 12)}}


def main():
    system = platform.system()
    supported = sys.version_info[:2] in SUPPORTED.get(system, set())
    report = {"cwd": str(Path.cwd()), "system": system,
              "platform": platform.platform(), "architecture": platform.machine(),
              "bits": struct.calcsize('P') * 8, "python": sys.version,
              "interpreter": sys.executable,
              "python_compatibility": "PASS" if supported else "WARN: check current README",
              "python_paths": {x: shutil.which(x) for x in ('python3.10', 'python3.12', 'python3.14', 'python', 'py')},
              "git": run_command(['git', '--version']),
              "pip": run_command([sys.executable, '-m', 'pip', '--version'])}
    if system == 'Linux':
        report['os_release'] = Path('/etc/os-release').read_text() if Path('/etc/os-release').exists() else 'unknown'
        result = run_command(['systemd-detect-virt'])
        report['virtualization'] = result
        report['interfaces'] = []
        for interface in Path('/sys/class/net').glob('*'):
            data = {'name': interface.name, 'physical_device': (interface / 'device').exists()}
            for key in ('mtu', 'operstate', 'carrier'):
                try:
                    data[key] = (interface / key).read_text().strip()
                except OSError:
                    data[key] = 'unknown'
            report['interfaces'].append(data)
    else:
        report['virtualization'] = 'unknown: inspect hypervisor/host manually'
    print(json.dumps(report, indent=2))
    return 0 if supported and report['bits'] == 64 else 1


if __name__ == '__main__':
    sys.exit(main())
