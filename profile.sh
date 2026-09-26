#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${BUILD_DIR:-${ROOT_DIR}/build-linux}"

cd "$ROOT_DIR"
cmake -S . -B "$BUILD_DIR" -DCMAKE_BUILD_TYPE=Release
cmake --build "$BUILD_DIR" --target OpenBus --parallel

export OPENBUS_VSYNC="${OPENBUS_VSYNC:-off}"
export OPENBUS_TRACE=1
export OPENBUS_TRACE_COLLAPSED=1
export OPENBUS_TRACE_FILE="${OPENBUS_TRACE_FILE:-${ROOT_DIR}/openbus_trace.json}"
export OPENBUS_TRACE_COLLAPSED_FILE="${OPENBUS_TRACE_COLLAPSED_FILE:-${ROOT_DIR}/openbus_trace.collapsed}"
exec "$BUILD_DIR/OpenBus"
