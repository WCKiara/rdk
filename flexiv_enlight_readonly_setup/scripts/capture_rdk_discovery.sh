#!/usr/bin/env bash
# Review before running. sudo is used only for a bounded, non-promiscuous
# discovery capture. The approved RDK diagnostic runs as the ordinary user.
set -u

if [ "$(id -u)" -eq 0 ]; then
    echo 'Run as the ordinary user; only tcpdump needs sudo.' >&2
    exit 2
fi
serial=${1:?Usage: bash scripts/capture_rdk_discovery.sh '<ACTUAL_FULL_SERIAL>'}
project_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
cd -- "$project_root" || exit 2
if [ ! -x .venv/bin/python ]; then
    echo 'Project virtualenv is missing.' >&2
    exit 2
fi
mkdir -p logs
echo 'Capturing eno2 UDP discovery and controller replies for 8 seconds; RDK reads info/states only.'
sudo -v || exit 2
sudo timeout 8 tcpdump -p -nn -tt -l -s 96 -i eno2 'udp and (host 192.168.2.100 or port 7446)' > logs/rdk_discovery_capture.txt 2>&1 &
capture_pid=$!
sleep 1
.venv/bin/python scripts/connect_readonly.py --serial "$serial" --connect-readonly --timeout-seconds 15
diagnostic_status=$?
wait "$capture_pid"
capture_status=$?
echo "RDK exit: $diagnostic_status; capture exit: $capture_status (124 means bounded timeout)."
tail -n 25 logs/rdk_discovery_capture.txt
exit "$diagnostic_status"
