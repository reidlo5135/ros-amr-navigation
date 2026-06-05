#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage: scripts/run_turtlebot3_nohup.sh [ros2 launch args...]

Start TurtleBot3 bringup under nohup for field tests.

Environment overrides:
  TURTLEBOT3_MODEL       TurtleBot3 model. Defaults to burger.
  AMR_TB3_PACKAGE        Launch package. Defaults to turtlebot3_bringup.
  AMR_TB3_LAUNCH         Launch file. Defaults to robot.launch.py.
  AMR_RUN_ID             Run id used in log and pid filenames.
  AMR_NOHUP_ROOT         Directory for logs and pid files.
EOF
}

if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
  usage
  exit 0
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=scripts/amr_logging_env.sh
source "${SCRIPT_DIR}/amr_logging_env.sh"

AMR_TB3_PACKAGE="${AMR_TB3_PACKAGE:-turtlebot3_bringup}"
AMR_TB3_LAUNCH="${AMR_TB3_LAUNCH:-robot.launch.py}"
LOG_FILE="${AMR_NOHUP_ROOT}/turtlebot3_${AMR_RUN_ID}.log"
PID_FILE="${AMR_NOHUP_ROOT}/turtlebot3_${AMR_RUN_ID}.pid"

cmd=(ros2 launch "${AMR_TB3_PACKAGE}" "${AMR_TB3_LAUNCH}" "$@")
amr_print_command "${cmd[@]}"
echo "[amr] TURTLEBOT3_MODEL=${TURTLEBOT3_MODEL}"
echo "[amr] log: ${LOG_FILE}"

nohup "${cmd[@]}" >"${LOG_FILE}" 2>&1 &
pid="$!"
printf '%s\n' "${pid}" >"${PID_FILE}"
echo "[amr] pid: ${pid}"
echo "[amr] pid_file: ${PID_FILE}"
