#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
INSTALL_BASE="/mnt/data/kiara/isaacsim"

if [[ ! -w "$INSTALL_BASE" ]]; then
    echo "数据盘安装目录不可写：$INSTALL_BASE" >&2
    exit 2
fi

mkdir -p "$INSTALL_BASE/logs"
if [[ ! -f "$INSTALL_BASE/installation.json" || ! -x "$INSTALL_BASE/6.0.0/python.sh" ]]; then
    python3 "$INSTALL_BASE/setup/rdk_install_isaac_sim.py" \
        2>&1 | tee -a "$INSTALL_BASE/logs/install.log"
fi

"$PROJECT_DIR/run_isaac_offline.sh" "$@" \
    2>&1 | tee -a "$INSTALL_BASE/logs/offline-replay.log"
