"""Bounded diagnostics only: no network edits, scanning, or robot API."""
import argparse
import ipaddress
import json
import math
import platform
import sys
from pathlib import Path
from common import positive_seconds, run_command, validate_ip


def ping_command(address, timeout, system):
    address = validate_ip(address)
    timeout = positive_seconds(timeout)
    family = '-6' if ipaddress.ip_address(address).version == 6 else '-4'
    if system == 'Windows':
        return ['ping', family, '-n', '4', '-w', str(math.ceil(timeout * 1000)), address]
    if system == 'Linux':
        return ['ping', family, '-n', '-c', '4', '-W', str(math.ceil(timeout)), address]
    raise ValueError('Unsupported ping platform; run platform tools manually')


def classify(result):
    if result.get('error') == 'timeout':
        return 'NO_REPLY: bounded command timed out'
    if result.get('error'):
        return 'UNAVAILABLE: ' + result['error']
    text = (result.get('stdout', '') + result.get('stderr', '')).lower()
    if any(x in text for x in ('operation not permitted', 'permission denied')):
        return 'UNAVAILABLE: execution permissions restrict diagnostics'
    if any(x in text for x in ('network is unreachable', 'no route', 'destination host unreachable', 'destination net unreachable')):
        return 'NO_ROUTE_OR_UNRESOLVED_PEER'
    if any(x in text for x in ('unknown host', 'name or service not known', 'could not find host')):
        return 'INVALID_ADDRESS_OR_DNS'
    if result['code'] == 0:
        return 'PING_SUCCESS: does not verify RDK, licensing, or Remote mode'
    return 'NO_REPLY: check route, subnet, power, ICMP filtering'


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--robot-ip', help='Actual operator-verified IP; no default')
    parser.add_argument('--timeout', type=positive_seconds, default=2)
    parser.add_argument('--output', type=Path, help='Optional local JSON containing network details')
    args = parser.parse_args(argv)
    system = platform.system()
    report = {'system': system, 'status': 'PASSIVE_ONLY', 'commands': []}
    commands = ([['ip', '-br', 'addr'], ['ip', 'route'], ['ip', 'link', 'show']]
                if system == 'Linux' else
                [['powershell', '-NoProfile', '-Command', 'Get-NetAdapter; Get-NetIPAddress -AddressFamily IPv4; Get-NetIPInterface -AddressFamily IPv4']]
                if system == 'Windows' else [])
    report['commands'] = [run_command(c) for c in commands]
    report['hint'] = 'If sandboxed, run this script in a normal terminal on the Python execution computer. Inspect the selected route and interface. For a VM, check both host and guest.'
    code = 0
    if not commands or all(item['code'] != 0 for item in report['commands']):
        report['status'] = 'WARN_UNAVAILABLE: interface/route inspection restricted; use the displayed platform commands in a normal terminal'
        code = 1
    if args.robot_ip:
        try:
            address = validate_ip(args.robot_ip)
            if system == 'Linux':
                report['route_to_robot'] = run_command(['ip', '-6' if ':' in address else '-4', 'route', 'get', address])
            result = run_command(ping_command(address, args.timeout, system), 4 * args.timeout + 6)
            report['ping'] = result
            report['status'] = classify(result)
            code = 0 if report['status'].startswith('PING_SUCCESS') else 1
        except ValueError as exc:
            report['status'] = 'INVALID_ADDRESS: ' + str(exc)
            code = 2
    rendered = json.dumps(report, indent=2)
    print(rendered)
    if args.output:
        args.output.write_text(rendered + '\n', encoding='utf-8')
    return code


if __name__ == '__main__':
    sys.exit(main())
