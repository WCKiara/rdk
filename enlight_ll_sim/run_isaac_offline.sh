#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ISAAC_SIM_ROOT="${ISAAC_SIM_ROOT:-/mnt/data/kiara/isaacsim/6.0.0}"

for argument in "$@"; do
    if [[ "$argument" == "--check-bundle" ]]; then
        exec python3 "$PROJECT_DIR/isaac_replay.py" --bundle "$PROJECT_DIR/isaac_bundle" "$@"
    fi
done

if [[ ! -x "$ISAAC_SIM_ROOT/python.sh" ]]; then
    echo "找不到 Isaac Sim：$ISAAC_SIM_ROOT/python.sh" >&2
    echo "请先完成安装，或用 ISAAC_SIM_ROOT 指定安装目录。" >&2
    exit 2
fi

if ! nvidia-smi --query-gpu=name --format=csv,noheader >/dev/null 2>&1; then
    echo "NVIDIA 驱动当前不可用，Isaac Sim 尚未启动。" >&2
    echo "请检查当前内核和 NVIDIA 模块；本机已安装 6.8.0-138-generic 内核。" >&2
    exit 3
fi

unset PYTHONHOME PYTHONPATH VIRTUAL_ENV CONDA_PREFIX
export ROS_LOCALHOST_ONLY=1
export OMNI_KIT_ACCEPT_EULA=YES
export XDG_CACHE_HOME="${XDG_CACHE_HOME:-/mnt/data/kiara/isaacsim/cache}"
export XDG_CONFIG_HOME="${XDG_CONFIG_HOME:-/mnt/data/kiara/isaacsim/config}"
mkdir -p "$XDG_CACHE_HOME" "$XDG_CONFIG_HOME"

echo "启动 Enlight-LL 离线参考轨迹回放，不导入 Flexiv SDK 或连接真机。"
cd "$ISAAC_SIM_ROOT"
exec "$ISAAC_SIM_ROOT/python.sh" "$PROJECT_DIR/isaac_replay.py" \
    --bundle "$PROJECT_DIR/isaac_bundle" "$@"
