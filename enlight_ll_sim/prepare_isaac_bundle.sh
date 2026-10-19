#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
lab_root="$(cd -- "$script_dir/../.." && pwd)"
description_dir="${FLEXIV_DESCRIPTION_DIR:-$HOME/flexiv_description}"
model_sn="${OFFLINE_MODEL_SN:-Enlight-LL-123456}"

if [[ ! -f "$description_dir/urdf/flexiv.urdf.xacro" ]]; then
    echo "Missing flexiv_description at $description_dir" >&2
    exit 1
fi

docker run --rm -i \
    --user "$(id -u):$(id -g)" \
    -v "$description_dir:/workspaces/src/flexiv_description:ro" \
    -v "$lab_root:/lab:rw" \
    -e "OFFLINE_MODEL_SN=$model_sn" \
    -w /workspaces \
    urdf_creation:latest \
    bash -lc 'colcon build --packages-select flexiv_description > /dev/null && source /workspaces/install/setup.bash && python3 /lab/rdk/enlight_ll_sim/export_isaac_bundle.py "$@"' bash "$@"
