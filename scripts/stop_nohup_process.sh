#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage: scripts/stop_nohup_process.sh [--label navigation|turtlebot3|all] [--pid-file FILE]

Stop nohup processes started by the AMR field scripts.

Options:
  --label NAME      Stop pid files matching NAME. Defaults to all.
  --pid-file FILE   Stop one explicit pid file.
  --help            Show this help.
EOF
}

label="all"
pid_file=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --label)
      label="${2:?missing value for --label}"
      shift 2
      ;;
    --pid-file)
      pid_file="${2:?missing value for --pid-file}"
      shift 2
      ;;
    --help|-h)
      usage
      exit 0
      ;;
    *)
      echo "unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=scripts/amr_logging_env.sh
source "${SCRIPT_DIR}/amr_logging_env.sh"

stop_pid_file() {
  local file="$1"
  if [[ ! -f "${file}" ]]; then
    echo "[amr] pid file not found: ${file}"
    return 0
  fi

  local pid
  pid="$(tr -d '[:space:]' <"${file}")"
  if [[ -z "${pid}" ]]; then
    echo "[amr] empty pid file: ${file}"
    rm -f "${file}"
    return 0
  fi

  if kill -0 "${pid}" >/dev/null 2>&1; then
    echo "[amr] stopping pid ${pid} from ${file}"
    kill "${pid}" >/dev/null 2>&1 || true
  else
    echo "[amr] pid ${pid} is not running"
  fi
  rm -f "${file}"
}

if [[ -n "${pid_file}" ]]; then
  stop_pid_file "${pid_file}"
  exit 0
fi

shopt -s nullglob
case "${label}" in
  all)
    files=("${AMR_NOHUP_ROOT}"/*.pid)
    ;;
  navigation|turtlebot3)
    files=("${AMR_NOHUP_ROOT}/${label}_"*.pid)
    ;;
  *)
    echo "unknown label: ${label}" >&2
    exit 2
    ;;
esac

if [[ ${#files[@]} -eq 0 ]]; then
  echo "[amr] no pid files matched label=${label} in ${AMR_NOHUP_ROOT}"
  exit 0
fi

for file in "${files[@]}"; do
  stop_pid_file "${file}"
done
