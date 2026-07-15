#!/usr/bin/env bash
set -euo pipefail

mode="${1:-check}"

check_interfaces() {
  ros2 lifecycle get /ft_navigator
  ros2 action list -t
  ros2 topic list
  ros2 service list
  ros2 topic info -v /frontier/status
  ros2 topic info -v /frontier/unknown_goal
  ros2 topic info -v /frontier/known_goal
  ros2 topic info -v /frontier/global_plan
  ros2 topic info -v /frontier/local_plan
  ros2 topic echo --once /map
  timeout 5 ros2 run tf2_ros tf2_echo map base_footprint || [ "$?" -eq 124 ]
}

record_bag() {
  output="${2:-frontier_navigation_$(date +%Y%m%d_%H%M%S)}"
  ros2 bag record --include-hidden-topics -o "${output}" \
    /map /tf /tf_static /scan \
    /global_costmap /local_costmap /global_plan /local_plan \
    /frontier/unknown_goal /frontier/known_goal \
    /frontier/global_plan /frontier/local_plan /frontier/status \
    /motion_status /cmd_vel \
    /navigate_to_unknown_pose/_action/feedback \
    /navigate_to_unknown_pose/_action/status \
    /navigate_to_unknown_pose/_action/send_goal \
    /navigate_to_unknown_pose/_action/get_result \
    /navigate_to_unknown_pose/_action/cancel_goal \
    /navigate_to_pose/_action/feedback \
    /navigate_to_pose/_action/status \
    /navigate_to_pose/_action/send_goal \
    /navigate_to_pose/_action/get_result \
    /navigate_to_pose/_action/cancel_goal
}

case "${mode}" in
  check)
    check_interfaces
    ;;
  record)
    record_bag "$@"
    ;;
  *)
    echo "Usage: $0 [check|record [bag_name]]" >&2
    exit 2
    ;;
esac
