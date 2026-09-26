#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${BUILD_DIR:-${ROOT_DIR}/build-linux}"
CONFIG_PATH="${1:-${ROOT_DIR}/MAN_DL05/Model/DL05.cfg}"
OUTPUT_DIR="${2:-${ROOT_DIR}/MAN_DL05/Model/DL05_obj}"

cd "$ROOT_DIR"
cmake -S . -B "$BUILD_DIR" -DCMAKE_BUILD_TYPE=Release
cmake --build "$BUILD_DIR" --target uno3d_converter --parallel
"$BUILD_DIR/uno3d_converter" --convert-textures "$CONFIG_PATH" --out "$OUTPUT_DIR"
