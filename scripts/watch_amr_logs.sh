#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage: scripts/watch_amr_logs.sh [--event EVENT] [--component COMPONENT] [--file FILE] [--no-follow] [--lines N]

Watch AMR_LOG lines from nohup logs.

Examples:
  scripts/watch_amr_logs.sh --event recovery_decision
  scripts/watch_amr_logs.sh --component controller --event cmd_quality --no-follow
EOF
}

event_filter=""
component_filter=""
log_file=""
follow=true
lines=200

while [[ $# -gt 0 ]]; do
  case "$1" in
    --event)
      event_filter="${2:?missing value for --event}"
      shift 2
      ;;
    --component)
      component_filter="${2:?missing value for --component}"
      shift 2
      ;;
    --file)
      log_file="${2:?missing value for --file}"
      shift 2
      ;;
    --no-follow)
      follow=false
      shift
      ;;
    --lines)
      lines="${2:?missing value for --lines}"
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

shopt -s nullglob
if [[ -n "${log_file}" ]]; then
  files=("${log_file}")
else
  files=("${AMR_NOHUP_ROOT}"/*.log)
fi

if [[ ${#files[@]} -eq 0 ]]; then
  echo "[amr] no log files found in ${AMR_NOHUP_ROOT}" >&2
  exit 2
fi

echo "[amr] logs: ${files[*]}"
echo "[amr] filter: component=${component_filter:-*} event=${event_filter:-*} follow=${follow}"

if [[ "${follow}" == "true" ]]; then
  tail -n "${lines}" -F "${files[@]}" \
    | grep --line-buffered 'AMR_LOG' \
    | { [[ -z "${component_filter}" ]] && cat || grep --line-buffered "component=${component_filter}"; } \
    | { [[ -z "${event_filter}" ]] && cat || grep --line-buffered "event=${event_filter}"; }
else
  grep -h 'AMR_LOG' "${files[@]}" \
    | { [[ -z "${component_filter}" ]] && cat || grep "component=${component_filter}"; } \
    | { [[ -z "${event_filter}" ]] && cat || grep "event=${event_filter}"; }
fi
