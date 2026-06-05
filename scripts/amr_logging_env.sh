#!/usr/bin/env bash
set -euo pipefail

amr_env_help() {
  cat <<'EOF'
Usage: source scripts/amr_logging_env.sh
       scripts/amr_logging_env.sh --print

Common environment for AMR field logging scripts.

Environment overrides:
  AMR_REPO_DIR       Repository directory. Defaults to this script's parent.
  AMR_WS_DIR         ROS workspace directory. Defaults to the parent workspace.
  AMR_ROS_DISTRO     ROS distro to source. Defaults to humble.
  AMR_LOG_ROOT       Root directory for text logs. Defaults to ${AMR_WS_DIR}/log/field.
  AMR_BAG_ROOT       Root directory for rosbag2 output. Defaults to ${AMR_WS_DIR}/bags.
  AMR_NOHUP_ROOT     Root directory for nohup logs and pid files.
  AMR_RUN_ID         Run identifier. Defaults to YYYYMMDD_HHMMSS.
  ROS_DOMAIN_ID      ROS domain id. Defaults to 0 if unset.
  TURTLEBOT3_MODEL   TurtleBot3 model. Defaults to burger if unset.
EOF
}

if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
  amr_env_help
  return 0 2>/dev/null || exit 0
fi

AMR_ENV_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export AMR_REPO_DIR="${AMR_REPO_DIR:-$(cd "${AMR_ENV_SCRIPT_DIR}/.." && pwd)}"

if [[ -z "${AMR_WS_DIR:-}" ]]; then
  if [[ -f "${AMR_REPO_DIR}/../../install/setup.bash" ]]; then
    AMR_WS_DIR="$(cd "${AMR_REPO_DIR}/../.." && pwd)"
  else
    AMR_WS_DIR="${AMR_REPO_DIR}"
  fi
fi
export AMR_WS_DIR

export AMR_ROS_DISTRO="${AMR_ROS_DISTRO:-humble}"
export AMR_LOG_ROOT="${AMR_LOG_ROOT:-${AMR_WS_DIR}/log/field}"
export AMR_BAG_ROOT="${AMR_BAG_ROOT:-${AMR_WS_DIR}/bags}"
export AMR_NOHUP_ROOT="${AMR_NOHUP_ROOT:-${AMR_LOG_ROOT}/nohup}"
export AMR_RUN_ID="${AMR_RUN_ID:-$(date +%Y%m%d_%H%M%S)}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export TURTLEBOT3_MODEL="${TURTLEBOT3_MODEL:-burger}"
export RCUTILS_LOGGING_USE_STDOUT="${RCUTILS_LOGGING_USE_STDOUT:-1}"
export RCUTILS_LOGGING_BUFFERED_STREAM="${RCUTILS_LOGGING_BUFFERED_STREAM:-1}"
export PYTHONUNBUFFERED="${PYTHONUNBUFFERED:-1}"

mkdir -p "${AMR_LOG_ROOT}" "${AMR_BAG_ROOT}" "${AMR_NOHUP_ROOT}"

amr_source_setup() {
  local setup_file="$1"
  set +u
  # shellcheck source=/dev/null
  source "${setup_file}"
  set -u
}

if [[ -f "/opt/ros/${AMR_ROS_DISTRO}/setup.bash" ]]; then
  amr_source_setup "/opt/ros/${AMR_ROS_DISTRO}/setup.bash"
else
  echo "[amr-env] warning: /opt/ros/${AMR_ROS_DISTRO}/setup.bash not found" >&2
fi

if [[ -f "${AMR_WS_DIR}/install/setup.bash" ]]; then
  amr_source_setup "${AMR_WS_DIR}/install/setup.bash"
else
  echo "[amr-env] warning: ${AMR_WS_DIR}/install/setup.bash not found" >&2
fi

amr_print_env() {
  cat <<EOF
AMR_REPO_DIR=${AMR_REPO_DIR}
AMR_WS_DIR=${AMR_WS_DIR}
AMR_ROS_DISTRO=${AMR_ROS_DISTRO}
AMR_LOG_ROOT=${AMR_LOG_ROOT}
AMR_BAG_ROOT=${AMR_BAG_ROOT}
AMR_NOHUP_ROOT=${AMR_NOHUP_ROOT}
AMR_RUN_ID=${AMR_RUN_ID}
ROS_DOMAIN_ID=${ROS_DOMAIN_ID}
TURTLEBOT3_MODEL=${TURTLEBOT3_MODEL}
EOF
}

amr_print_command() {
  printf '[amr] command:'
  printf ' %q' "$@"
  printf '\n'
}

if [[ "${1:-}" == "--print" || "${1:-}" == "" && "${BASH_SOURCE[0]}" == "$0" ]]; then
  amr_print_env
fi
