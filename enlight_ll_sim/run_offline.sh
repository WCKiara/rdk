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

motion_args_json="$(python3 -c 'import json, sys; print(json.dumps(sys.argv[1:]))' "$@")"
check_only=0
for arg in "$@"; do
    if [[ "$arg" == "--check-only" ]]; then
        check_only=1
    fi
done

docker run --rm -i \
    --user "$(id -u):$(id -g)" \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v "$description_dir:/workspaces/src/flexiv_description:ro" \
    -v "$lab_root:/lab:ro" \
    -e "DISPLAY=${DISPLAY:-:0}" \
    -e "OFFLINE_MOTION_ARGS_JSON=$motion_args_json" \
    -e "OFFLINE_MODEL_SN=$model_sn" \
    -e "OFFLINE_CHECK_ONLY=$check_only" \
    -e QT_X11_NO_MITSHM=1 \
    -e QT_SCALE_FACTOR=1 \
    -e LIBGL_ALWAYS_SOFTWARE=1 \
    -w /workspaces \
    urdf_creation:latest \
    bash -lc 'colcon build --packages-select flexiv_description > /dev/null && source /workspaces/install/setup.bash && if [[ "$OFFLINE_CHECK_ONLY" == 1 ]]; then python3 /lab/rdk/enlight_ll_sim/play_intermediate4.py "$@"; else ros2 launch /lab/rdk/enlight_ll_sim/offline_viewer.launch.py robot_sn:="$OFFLINE_MODEL_SN"; fi' bash "$@"
