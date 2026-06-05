#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage: scripts/run_navigation_nohup.sh [ros2 launch args...]

Start the AMR navigation launch under nohup and collect stdout/stderr.

Environment overrides:
  AMR_NAV_PACKAGE   Launch package. Defaults to amr_bringup.
  AMR_NAV_LAUNCH    Launch file. Defaults to navigation.launch.py.
  AMR_RUN_ID        Run id used in log and pid filenames.
  AMR_NOHUP_ROOT    Directory for logs and pid files.
EOF
}

if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
  usage
  exit 0
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=scripts/amr_logging_env.sh
source "${SCRIPT_DIR}/amr_logging_env.sh"

AMR_NAV_PACKAGE="${AMR_NAV_PACKAGE:-amr_bringup}"
AMR_NAV_LAUNCH="${AMR_NAV_LAUNCH:-navigation.launch.py}"
LOG_FILE="${AMR_NOHUP_ROOT}/navigation_${AMR_RUN_ID}.log"
PID_FILE="${AMR_NOHUP_ROOT}/navigation_${AMR_RUN_ID}.pid"

cmd=(ros2 launch "${AMR_NAV_PACKAGE}" "${AMR_NAV_LAUNCH}" "$@")
amr_print_command "${cmd[@]}"
echo "[amr] log: ${LOG_FILE}"

nohup "${cmd[@]}" >"${LOG_FILE}" 2>&1 &
pid="$!"
printf '%s\n' "${pid}" >"${PID_FILE}"
echo "[amr] pid: ${pid}"
echo "[amr] pid_file: ${PID_FILE}"
