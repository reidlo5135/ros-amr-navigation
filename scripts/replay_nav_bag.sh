#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage: scripts/replay_nav_bag.sh BAG_PATH [ros2 bag play args...]

Replay a navigation rosbag with simulated clock enabled.
EOF
}

if [[ "${1:-}" == "--help" || "${1:-}" == "-h" || $# -lt 1 ]]; then
  usage
  [[ $# -lt 1 && "${1:-}" != "--help" && "${1:-}" != "-h" ]] && exit 2
  exit 0
fi

bag_path="$1"
shift

if [[ ! -e "${bag_path}" ]]; then
  echo "bag path not found: ${bag_path}" >&2
  exit 2
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=scripts/amr_logging_env.sh
source "${SCRIPT_DIR}/amr_logging_env.sh"

cmd=(ros2 bag play "${bag_path}" --clock "$@")
amr_print_command "${cmd[@]}"
"${cmd[@]}"
