#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
SDK_DIR="${OPENNI2_SDK_DIR:-$SCRIPT_DIR/.sdk_cache/arm64_sdk/OpenNI_2.3.0.86_202210111155_4c8f5aa4_beta6_a311d}"
INCLUDE_DIR="$SDK_DIR/sdk/Include"
LIB_DIR="$SDK_DIR/sdk/libs"
OUTPUT_FILE="${1:-$SCRIPT_DIR/camera_calibration.txt}"
PROGRAM="$SCRIPT_DIR/read_camera_calibration"

if [[ ! -f "$INCLUDE_DIR/OpenNI.h" || ! -f "$LIB_DIR/libOpenNI2.so" ]]; then
    echo "OpenNI SDK headers/library not found under: $SDK_DIR" >&2
    echo "Set OPENNI2_SDK_DIR to the SDK root directory if it is elsewhere." >&2
    exit 1
fi

g++ -std=c++11 -O2 -Wall -Wextra \
    -I"$INCLUDE_DIR" \
    "$SCRIPT_DIR/read_camera_calibration.cpp" \
    -L"$LIB_DIR" "-Wl,-rpath,$LIB_DIR" -lOpenNI2 \
    -o "$PROGRAM"

export OPENNI2_REDIST="$LIB_DIR"
"$PROGRAM" "$OUTPUT_FILE"

echo "Calibration TXT: $OUTPUT_FILE"
