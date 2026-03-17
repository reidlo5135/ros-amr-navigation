#!/usr/bin/env bash

set -euo pipefail

if [[ $# -lt 3 ]]; then
  echo "usage: $0 <x> <y> <yaw> [frame_id] [action_name]" >&2
  exit 1
fi

x="$1"
y="$2"
yaw="$3"
frame_id="${4:-map}"
action_name="${5:-/amr/navigator/navigate_to_pose}"

read -r qz qw < <(awk -v yaw="$yaw" 'BEGIN { printf "%.16f %.16f\n", sin(yaw / 2.0), cos(yaw / 2.0) }')

ros2 action send_goal "${action_name}" amr_msgs/action/NavigateToPose "{
  goal_pose: {
    header: {frame_id: ${frame_id}},
    pose: {
      position: {x: ${x}, y: ${y}, z: 0.0},
      orientation: {x: 0.0, y: 0.0, z: ${qz}, w: ${qw}}
    }
  }
}" --feedback
