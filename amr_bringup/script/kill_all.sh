#!/usr/bin/env bash

set -euo pipefail

kill_if_running() {
  local pattern="$1"
  local label="$2"

  if pgrep -f "${pattern}" >/dev/null 2>&1; then
    pkill -f "${pattern}" >/dev/null 2>&1 || true
    echo "Stopped ${label}"
  fi
}

echo "Stopping AMR Viz / ROS related processes..."

kill_if_running "ros2 launch amr_viz" "amr_viz launch"
kill_if_running "ros2 launch amr_bringup" "amr_bringup launch"
kill_if_running "amr_viz_bridge" "amr_viz bridge"
kill_if_running "vite --host 0.0.0.0 --port" "vite dev server"
kill_if_running "vite --host 127.0.0.1 --port" "vite dev server"
kill_if_running "vite preview --host 0.0.0.0 --port" "vite preview server"
kill_if_running "electron ." "electron app"
kill_if_running "amr_goal_bridge" "rviz goal bridge"
kill_if_running "rviz2" "rviz2"
kill_if_running "/home/reidlo/ws/install/amr_viz" "installed amr_viz python process"
kill_if_running "/home/reidlo/ws/src/ros-amr-navigation/install/amr_viz" "legacy installed amr_viz python process"

echo "Done."
