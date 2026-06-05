#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage: scripts/extract_nav_quality.sh [--file FILE] [--dir DIR] [--output FILE] [--format tsv|csv]

Extract navigation quality AMR_LOG events from text logs.

Default events:
  goal_state, cmd_quality, tracking_state, tracking_heading_debug,
  tracking_frame_mismatch, target_jump_detected, recovery_decision

Environment overrides:
  AMR_NOHUP_ROOT   Default log directory.
EOF
}

log_file=""
log_dir=""
output_file=""
format="tsv"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --file)
      log_file="${2:?missing value for --file}"
      shift 2
      ;;
    --dir)
      log_dir="${2:?missing value for --dir}"
      shift 2
      ;;
    --output)
      output_file="${2:?missing value for --output}"
      shift 2
      ;;
    --format)
      format="${2:?missing value for --format}"
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

if [[ "${format}" != "tsv" && "${format}" != "csv" ]]; then
  echo "unsupported format: ${format}" >&2
  exit 2
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=scripts/amr_logging_env.sh
source "${SCRIPT_DIR}/amr_logging_env.sh"

shopt -s nullglob
if [[ -n "${log_file}" ]]; then
  files=("${log_file}")
else
  search_dir="${log_dir:-${AMR_NOHUP_ROOT}}"
  files=("${search_dir}"/*.log)
fi

if [[ ${#files[@]} -eq 0 ]]; then
  echo "[amr] no log files found" >&2
  exit 2
fi

extract_cmd=(awk -v format="${format}" '
function value_of(entry, key, pattern, raw) {
  pattern = "(^| )" key "=[^ ]+"
  if (match(entry, pattern)) {
    raw = substr(entry, RSTART, RLENGTH)
    sub(/^ /, "", raw)
    sub("^" key "=", "", raw)
    return raw
  }
  return ""
}
function emit(fields, raw, i) {
  if (format == "csv") {
    for (i = 1; i <= 25; i++) {
      gsub(/"/, "\"\"", fields[i])
      printf "%s\"%s\"", (i == 1 ? "" : ","), fields[i]
    }
    gsub(/"/, "\"\"", raw)
    printf ",\"%s\"\n", raw
  } else {
    for (i = 1; i <= 25; i++) {
      gsub(/\t/, " ", fields[i])
      printf "%s%s", (i == 1 ? "" : "\t"), fields[i]
    }
    gsub(/\t/, " ", raw)
    printf "\t%s\n", raw
  }
}
BEGIN {
  if (format == "csv") {
    print "source,component,event,phase,goal_id,target_idx,selected_idx,nearest_idx,candidate_idx,target_dist_m,target_jump_m,dist_goal_m,heading_err_rad,steering_err_rad,cmd_lin,cmd_ang,output_lin,output_ang,recovery_count,blocked,reason,selection_reason,pose_frame,plan_frame,target_frame,raw"
  } else {
    print "source\tcomponent\tevent\tphase\tgoal_id\ttarget_idx\tselected_idx\tnearest_idx\tcandidate_idx\ttarget_dist_m\ttarget_jump_m\tdist_goal_m\theading_err_rad\tsteering_err_rad\tcmd_lin\tcmd_ang\toutput_lin\toutput_ang\trecovery_count\tblocked\treason\tselection_reason\tpose_frame\tplan_frame\ttarget_frame\traw"
  }
}
/AMR_LOG/ {
  pos = index($0, "AMR_LOG")
  entry = substr($0, pos)
  event = value_of(entry, "event")
  if (event != "goal_state" && event != "cmd_quality" && event != "tracking_state" && event != "tracking_heading_debug" && event != "tracking_frame_mismatch" && event != "target_jump_detected" && event != "recovery_decision") {
    next
  }
  target_idx = value_of(entry, "target_idx")
  selected_idx = value_of(entry, "selected_idx")
  if (target_idx == "") {
    target_idx = selected_idx
  }
  fields[1] = FILENAME
  fields[2] = value_of(entry, "component")
  fields[3] = event
  fields[4] = value_of(entry, "phase")
  fields[5] = value_of(entry, "goal_id")
  fields[6] = target_idx
  fields[7] = selected_idx
  fields[8] = value_of(entry, "nearest_idx")
  fields[9] = value_of(entry, "candidate_idx")
  fields[10] = value_of(entry, "target_dist_m")
  fields[11] = value_of(entry, "target_jump_m")
  fields[12] = value_of(entry, "dist_goal_m")
  fields[13] = value_of(entry, "heading_err_rad")
  fields[14] = value_of(entry, "steering_err_rad")
  fields[15] = value_of(entry, "cmd_lin")
  fields[16] = value_of(entry, "cmd_ang")
  fields[17] = value_of(entry, "output_lin")
  fields[18] = value_of(entry, "output_ang")
  fields[19] = value_of(entry, "recovery_count")
  fields[20] = value_of(entry, "blocked")
  fields[21] = value_of(entry, "reason")
  fields[22] = value_of(entry, "selection_reason")
  fields[23] = value_of(entry, "pose_frame")
  fields[24] = value_of(entry, "plan_frame")
  fields[25] = value_of(entry, "target_frame")
  emit(fields, entry)
}
' "${files[@]}")

echo "[amr] input logs: ${files[*]}" >&2
if [[ -n "${output_file}" ]]; then
  mkdir -p "$(dirname "${output_file}")"
  echo "[amr] command: awk <nav_quality_parser> ${files[*]} > ${output_file}" >&2
  "${extract_cmd[@]}" >"${output_file}"
  echo "[amr] wrote: ${output_file}"
else
  echo "[amr] command: awk <nav_quality_parser> ${files[*]}" >&2
  "${extract_cmd[@]}"
fi
