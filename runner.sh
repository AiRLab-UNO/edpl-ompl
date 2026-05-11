set -uo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="$PROJECT_ROOT/build"
FIRMCP_BIN="$BUILD_DIR/firmcp-demo"

$FIRMCP_BIN $PROJECT_ROOT/examples/configs/firmcp/SetupFIRMCP-Beacon.yaml