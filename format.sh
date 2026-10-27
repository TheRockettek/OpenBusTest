#!/usr/bin/env bash
set -uo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if ! command -v clang-format >/dev/null 2>&1; then
    echo "clang-format was not found on PATH." >&2
    echo "Install LLVM or add its bin directory to PATH." >&2
    exit 1
fi

find "$ROOT_DIR/src" "$ROOT_DIR/uno3d" -type f \( -name '*.cpp' -o -name '*.h' \) -print0 |
    while IFS= read -r -d '' file; do
        echo "Formatting $file"
        if ! clang-format -i "$file"; then
            echo "clang-format failed for $file." >&2
            exit 1
        fi
    done
