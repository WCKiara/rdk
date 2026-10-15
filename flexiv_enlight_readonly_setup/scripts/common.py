"""Shared validation and bounded, read-only command execution."""
import ipaddress
import re
import subprocess


def positive_seconds(value):
    import math
    value = float(value)
    if not math.isfinite(value) or value <= 0:
        raise ValueError("Timeout must be finite and positive")
    return value


def validate_serial(value):
    # Accept the official model-prefix convention without inventing a digit count.
    if not value or not re.fullmatch(r"[A-Za-z0-9]+(?:-[A-Za-z0-9]+)+", value):
        raise ValueError("Supply the actual hyphenated serial from Elements/label; no spaces")
    if value.lower() == "enlight-l-123456" or any(
        word in value.upper() for word in ("REPLACE", "EXAMPLE", "ACTUAL", "PLACEHOLDER")
    ):
        raise ValueError("Example/placeholder serial is forbidden")
    return value


def validate_ip(value):
    try:
        address = ipaddress.ip_address(value)
    except ValueError as exc:
        raise ValueError("Invalid IP address; use a verified literal address, not DNS") from exc
    if address.is_unspecified or address.is_multicast or address.is_loopback:
        raise ValueError("Robot address must be a unicast peer address")
    if str(address) == "255.255.255.255":
        raise ValueError("Broadcast address is forbidden")
    return str(address)


def run_command(argv, timeout=10):
    try:
        result = subprocess.run(argv, capture_output=True, text=True, timeout=timeout)
        return {"command": argv, "code": result.returncode,
                "stdout": result.stdout.strip(), "stderr": result.stderr.strip()}
    except subprocess.TimeoutExpired:
        return {"command": argv, "code": None, "error": "timeout"}
    except OSError as exc:
        return {"command": argv, "code": None, "error": str(exc)}
