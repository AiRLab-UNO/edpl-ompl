#!/bin/bash
# set -uo pipefail
#
# Usage (from project root):
#   bash runner.sh [config.yaml] [stl]
#
# The script cd's into /build so that all YAML-relative paths
# (../results/, ../examples/, ../run-*/) resolve correctly.
#
# Examples:
#   bash runner.sh                              # firmcp-demo + SetupFIRMCP.yaml
#   bash runner.sh SetupFIRMCP-Wall.yaml        # firmcp-demo + Wall config
#   bash runner.sh SetupFIRMCP.yaml stl         # stl-firmcp-demo + same config
#   bash runner.sh SetupFIRMCP-Beacon.yaml stl  # stl-firmcp-demo + Beacon config

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="$PROJECT_ROOT/build"

CONFIG="${1:-SetupFIRMCP.yaml}"
MODE="${2:-firmcp}"   # "stl" -> stl-firmcp-demo, anything else -> firmcp-demo

YAML_PATH="$PROJECT_ROOT/examples/configs/firmcp/$CONFIG"

cd "$BUILD_DIR" || { echo "ERROR: cannot cd to $BUILD_DIR"; exit 1; }

if [[ "$MODE" == "stl" ]]; then
    exec "$BUILD_DIR/stl-firmcp-demo" "$YAML_PATH"
else
    exec "$BUILD_DIR/firmcp-demo" "$YAML_PATH"
fi
