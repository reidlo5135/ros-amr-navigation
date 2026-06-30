#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage: scripts/record_nav_bag_light.sh [ros2 bag record args...]

Record a light navigation rosbag profile focused on runtime observation and
decision/status streams. This profile avoids scan, TF, and costmap grids.

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

output="${AMR_BAG_ROOT}/nav_light_${AMR_RUN_ID}"
topics=(
  /observation/runtime/summary
  /observation/runtime/events
  /motion_command
  /motion_status
  /local_plan_status
  /cmd_vel
)

cmd=(ros2 bag record --output "${output}" "${topics[@]}" "$@")
amr_print_command "${cmd[@]}"
"${cmd[@]}"
