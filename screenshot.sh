#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${BUILD_DIR:-${ROOT_DIR}/build-linux}"

cd "$ROOT_DIR"
cmake -S . -B "$BUILD_DIR" -DCMAKE_BUILD_TYPE=Release
cmake --build "$BUILD_DIR" --target OpenBus --parallel

export OPENBUS_VSYNC="${OPENBUS_VSYNC:-off}"
export OPENBUS_CAPTURE_VIEWS=1
exec "$BUILD_DIR/OpenBus"
