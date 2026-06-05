#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage: scripts/record_nav_bag_full.sh [ros2 bag record args...]

Record all visible ROS topics with rosbag2. Use this only for short, targeted
debug windows because it may be large.

Environment overrides:
  AMR_BAG_ROOT   Bag output root.
  AMR_RUN_ID     Run id used in the bag directory.
EOF
}

if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
  usage
  exit 0
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=scripts/amr_logging_env.sh
source "${SCRIPT_DIR}/amr_logging_env.sh"

output="${AMR_BAG_ROOT}/nav_full_${AMR_RUN_ID}"
cmd=(ros2 bag record --all --output "${output}" "$@")
amr_print_command "${cmd[@]}"
"${cmd[@]}"
