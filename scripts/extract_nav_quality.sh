#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage: scripts/extract_nav_quality.sh [--file FILE] [--dir DIR] [--output FILE] [--format tsv|csv]

Extract navigation quality AMR_LOG events from text logs.

Default events:
  goal_state, cmd_quality, tracking_state, tracking_heading_debug,
  tracking_frame_mismatch, local_path_quality, target_jump_detected,
  recovery_decision

The output also carries rejoin_context_active and appends one nav_quality_summary
row with straight-line angular interference metrics computed from cmd_quality rows.

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
function abs_value(value) {
  return value < 0 ? -value : value
}
function reset_fields(fields, i) {
  for (i = 1; i <= field_count; i++) {
    fields[i] = ""
  }
}
function emit(fields, raw, i) {
  if (format == "csv") {
    for (i = 1; i <= field_count; i++) {
      gsub(/"/, "\"\"", fields[i])
      printf "%s\"%s\"", (i == 1 ? "" : ","), fields[i]
    }
    gsub(/"/, "\"\"", raw)
    printf ",\"%s\"\n", raw
  } else {
    for (i = 1; i <= field_count; i++) {
      gsub(/\t/, " ", fields[i])
      printf "%s%s", (i == 1 ? "" : "\t"), fields[i]
    }
    gsub(/\t/, " ", raw)
    printf "\t%s\n", raw
  }
}
BEGIN {
  field_count = 52
  if (format == "csv") {
    print "source,component,event,phase,goal_id,target_idx,selected_idx,nearest_idx,candidate_idx,target_dist_m,target_jump_m,dist_goal_m,heading_err_rad,steering_err_rad,cmd_lin,cmd_ang,output_lin,output_ang,recovery_count,blocked,reason,selection_reason,pose_frame,plan_frame,target_frame,rejoin_context_active,straight_segment,path_curvature_score,lateral_error_m,heading_error_raw_rad,heading_error_filtered_rad,steering_deadband_active,steering_hysteresis_state,cmd_ang_sign,cmd_ang_flip_count,output_ang_sign,output_ang_flip_count,cmd_ang_abs_avg,cmd_ang_abs_max,output_ang_abs_avg,output_ang_abs_max,cmd_ang_sign_flip_count,output_ang_sign_flip_count,straight_segment_ratio,straight_cmd_ang_interference_count,raw_path_points,simplified_path_points,refined_path_points,path_length_m,line_of_sight_simplified,collinear_pruned_count,collision_check_passed,raw"
  } else {
    print "source\tcomponent\tevent\tphase\tgoal_id\ttarget_idx\tselected_idx\tnearest_idx\tcandidate_idx\ttarget_dist_m\ttarget_jump_m\tdist_goal_m\theading_err_rad\tsteering_err_rad\tcmd_lin\tcmd_ang\toutput_lin\toutput_ang\trecovery_count\tblocked\treason\tselection_reason\tpose_frame\tplan_frame\ttarget_frame\trejoin_context_active\tstraight_segment\tpath_curvature_score\tlateral_error_m\theading_error_raw_rad\theading_error_filtered_rad\tsteering_deadband_active\tsteering_hysteresis_state\tcmd_ang_sign\tcmd_ang_flip_count\toutput_ang_sign\toutput_ang_flip_count\tcmd_ang_abs_avg\tcmd_ang_abs_max\toutput_ang_abs_avg\toutput_ang_abs_max\tcmd_ang_sign_flip_count\toutput_ang_sign_flip_count\tstraight_segment_ratio\tstraight_cmd_ang_interference_count\traw_path_points\tsimplified_path_points\trefined_path_points\tpath_length_m\tline_of_sight_simplified\tcollinear_pruned_count\tcollision_check_passed\traw"
  }
}
/AMR_LOG/ {
  pos = index($0, "AMR_LOG")
  entry = substr($0, pos)
  event = value_of(entry, "event")
  if (event != "goal_state" && event != "cmd_quality" && event != "tracking_state" && event != "tracking_heading_debug" && event != "tracking_frame_mismatch" && event != "local_path_quality" && event != "target_jump_detected" && event != "recovery_decision") {
    next
  }
  target_idx = value_of(entry, "target_idx")
  selected_idx = value_of(entry, "selected_idx")
  if (target_idx == "") {
    target_idx = selected_idx
  }
  reset_fields(fields)
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
  fields[26] = value_of(entry, "rejoin_context_active")
  fields[27] = value_of(entry, "straight_segment")
  fields[28] = value_of(entry, "path_curvature_score")
  fields[29] = value_of(entry, "lateral_error_m")
  fields[30] = value_of(entry, "heading_error_raw_rad")
  fields[31] = value_of(entry, "heading_error_filtered_rad")
  fields[32] = value_of(entry, "steering_deadband_active")
  fields[33] = value_of(entry, "steering_hysteresis_state")
  fields[34] = value_of(entry, "cmd_ang_sign")
  fields[35] = value_of(entry, "cmd_ang_flip_count")
  fields[36] = value_of(entry, "output_ang_sign")
  fields[37] = value_of(entry, "output_ang_flip_count")
  fields[46] = value_of(entry, "raw_path_points")
  fields[47] = value_of(entry, "simplified_path_points")
  fields[48] = value_of(entry, "refined_path_points")
  fields[49] = value_of(entry, "path_length_m")
  fields[50] = value_of(entry, "line_of_sight_simplified")
  fields[51] = value_of(entry, "collinear_pruned_count")
  fields[52] = value_of(entry, "collision_check_passed")
  if (event == "cmd_quality") {
    cmd_ang = fields[16]
    output_ang = fields[18]
    if (cmd_ang != "") {
      cmd_abs = abs_value(cmd_ang + 0.0)
      cmd_ang_abs_sum += cmd_abs
      cmd_ang_abs_count += 1
      if (cmd_abs > cmd_ang_abs_max) {
        cmd_ang_abs_max = cmd_abs
      }
    }
    if (output_ang != "") {
      output_abs = abs_value(output_ang + 0.0)
      output_ang_abs_sum += output_abs
      output_ang_abs_count += 1
      if (output_abs > output_ang_abs_max) {
        output_ang_abs_max = output_abs
      }
    }
    if (fields[27] != "") {
      straight_segment_samples += 1
      if (fields[27] == "true") {
        straight_segment_count += 1
        if (cmd_ang != "" && abs_value(cmd_ang + 0.0) > 0.02) {
          straight_cmd_ang_interference_count += 1
        }
      }
    }
    if (fields[35] != "") {
      cmd_flip_key = fields[5] == "" ? FILENAME : fields[5]
      cmd_flip_value = fields[35] + 0
      if (!(cmd_flip_key in last_cmd_flip_count) || cmd_flip_value < last_cmd_flip_count[cmd_flip_key]) {
        cmd_ang_sign_flip_count += cmd_flip_value
      } else {
        cmd_ang_sign_flip_count += cmd_flip_value - last_cmd_flip_count[cmd_flip_key]
      }
      last_cmd_flip_count[cmd_flip_key] = cmd_flip_value
    }
    if (fields[37] != "") {
      output_flip_key = fields[5] == "" ? FILENAME : fields[5]
      output_flip_value = fields[37] + 0
      if (!(output_flip_key in last_output_flip_count) || output_flip_value < last_output_flip_count[output_flip_key]) {
        output_ang_sign_flip_count += output_flip_value
      } else {
        output_ang_sign_flip_count += output_flip_value - last_output_flip_count[output_flip_key]
      }
      last_output_flip_count[output_flip_key] = output_flip_value
    }
  }
  emit(fields, entry)
}
END {
  reset_fields(fields)
  fields[1] = "summary"
  fields[2] = "controller"
  fields[3] = "nav_quality_summary"
  fields[38] = cmd_ang_abs_count > 0 ? sprintf("%.6f", cmd_ang_abs_sum / cmd_ang_abs_count) : ""
  fields[39] = cmd_ang_abs_count > 0 ? sprintf("%.6f", cmd_ang_abs_max) : ""
  fields[40] = output_ang_abs_count > 0 ? sprintf("%.6f", output_ang_abs_sum / output_ang_abs_count) : ""
  fields[41] = output_ang_abs_count > 0 ? sprintf("%.6f", output_ang_abs_max) : ""
  fields[42] = sprintf("%d", cmd_ang_sign_flip_count)
  fields[43] = sprintf("%d", output_ang_sign_flip_count)
  fields[44] = straight_segment_samples > 0 ? sprintf("%.6f", straight_segment_count / straight_segment_samples) : ""
  fields[45] = sprintf("%d", straight_cmd_ang_interference_count)
  emit(fields, "AMR_LOG schema=v1 component=controller event=nav_quality_summary source=extract_nav_quality")
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
