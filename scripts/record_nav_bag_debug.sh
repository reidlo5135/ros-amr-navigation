#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage: scripts/record_nav_bag_debug.sh [ros2 bag record args...]

Record a debug navigation rosbag profile. Includes light profile topics plus
paths, local costmap, scan, odometry, and TF for deeper field diagnosis.

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

output="${AMR_BAG_ROOT}/nav_debug_${AMR_RUN_ID}"
topics=(
  /observation/runtime/summary
  /observation/runtime/events
  /motion_command
  /motion_status
  /local_plan_status
  /global_plan
  /local_plan
  /local_costmap
  /scan
  /tf
  /tf_static
  /cmd_vel
)

cmd=(ros2 bag record --output "${output}" "${topics[@]}" "$@")
amr_print_command "${cmd[@]}"
"${cmd[@]}"
