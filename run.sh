#!/usr/bin/env bash
set -euo pipefail

export OPENBUS_OMSI_ROOT="/home/rock/.local/share/Steam/steamapps/common/OMSI 2"
export OPENBUS_BUS_CONFIG="Vehicles/MAN_DL05/MAN_DL05.bus"
export OPENBUS_MODEL_CONFIG="Vehicles/MAN_DL05/Model/DL05.cfg"
export OPENBUS_MAP_PATH="London"
export OPENBUS_VSYNC="on"
export OPENBUS_REFLECTION_TRANSPARENT="0"
export OPENBUS_MATERIAL_BATCHING="1"
export OPENBUS_REFLECTION_MAX_FPS="30"
export OPENBUS_FRUSTUM_CULLING="1"
export OPENBUS_WHEELS_FROM_ODE="0"
export OPENBUS_SCRIPT_BACKEND="native"
export OPENBUS_SCRIPT_HZ="60"
export OPENBUS_ASSET_WORKERS="4"
export OPENBUS_DOPPLER="1"
export OPENBUS_CAPTURE_VIEWS=""

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${BUILD_DIR:-${ROOT_DIR}/build-linux}"

cd "$ROOT_DIR"
cmake -S . -B "$BUILD_DIR" -DCMAKE_BUILD_TYPE=Release -DOPENBUS_ENABLE_PERF_TRACE=OFF
cmake --build "$BUILD_DIR" --target OpenBus --parallel

export OPENBUS_VSYNC="${OPENBUS_VSYNC:-off}"
exec "$BUILD_DIR/OpenBus"
