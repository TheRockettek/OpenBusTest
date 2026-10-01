#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${BUILD_DIR:-${ROOT_DIR}/build-linux}"

cd "$ROOT_DIR"
cmake -S . -B "$BUILD_DIR" -DCMAKE_BUILD_TYPE=Release -DOPENBUS_ENABLE_PERF_TRACE=OFF
cmake --build "$BUILD_DIR" --target OpenBus --parallel

export OPENBUS_VSYNC="${OPENBUS_VSYNC:-off}"
exec "$BUILD_DIR/OpenBus"
