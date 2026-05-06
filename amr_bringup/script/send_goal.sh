#!/usr/bin/env bash

set -euo pipefail

ros2 action send_goal /amr/navigator/navigate_to_poses amr_msgs/action/NavigateToPoses "{
  goal_poses: [
    {
      header: {frame_id: 'map'},
      pose: {
        position: {x: 2.0, y: 0.0, z: 0.0},
        orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
      }
    },
    {
      header: {frame_id: 'map'},
      pose: {
        position: {x: 4.0, y: 0.0, z: 0.0},
        orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
      }
    },
    {
      header: {frame_id: 'map'},
      pose: {
        position: {x: 6.0, y: 0.0, z: 0.0},
        orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
      }
    },
    {
      header: {frame_id: 'map'},
      pose: {
        position: {x: 4.0, y: 0.0, z: 0.0},
        orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
      }
    },
    {
      header: {frame_id: 'map'},
      pose: {
        position: {x: 2.0, y: 0.0, z: 0.0},
        orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
      }
    },
    {
      header: {frame_id: 'map'},
      pose: {
        position: {x: 0.0, y: 0.0, z: 0.0},
        orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
      }
    }
  ]
}"
