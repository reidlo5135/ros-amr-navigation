import type { BridgeState } from "./protocol";

export const mockState: BridgeState = {
  robot_pose: {
    header: {
      stamp: { sec: 0, nanosec: 0 },
      frame_id: "map",
    },
    position: { x: 0, y: 0, z: 0 },
    orientation: { x: 0, y: 0, z: 0, w: 1, yaw: 0 },
  },
  global_path: {
    header: {
      stamp: { sec: 0, nanosec: 0 },
      frame_id: "map",
    },
    poses: [
      {
        header: { stamp: { sec: 0, nanosec: 0 }, frame_id: "map" },
        position: { x: 0, y: 0, z: 0 },
        orientation: { x: 0, y: 0, z: 0, w: 1, yaw: 0 },
      },
      {
        header: { stamp: { sec: 0, nanosec: 0 }, frame_id: "map" },
        position: { x: 1.4, y: 0.2, z: 0 },
        orientation: { x: 0, y: 0, z: 0, w: 1, yaw: 0 },
      },
      {
        header: { stamp: { sec: 0, nanosec: 0 }, frame_id: "map" },
        position: { x: 3.0, y: 0.2, z: 0 },
        orientation: { x: 0, y: 0, z: 0, w: 1, yaw: 0 },
      },
    ],
  },
  local_path: {
    header: {
      stamp: { sec: 0, nanosec: 0 },
      frame_id: "map",
    },
    poses: [
      {
        header: { stamp: { sec: 0, nanosec: 0 }, frame_id: "map" },
        position: { x: 0, y: 0, z: 0 },
        orientation: { x: 0, y: 0, z: 0, w: 1, yaw: 0 },
      },
      {
        header: { stamp: { sec: 0, nanosec: 0 }, frame_id: "map" },
        position: { x: 1.0, y: -0.35, z: 0 },
        orientation: { x: 0, y: 0, z: 0, w: 1, yaw: -0.15 },
      },
      {
        header: { stamp: { sec: 0, nanosec: 0 }, frame_id: "map" },
        position: { x: 2.2, y: -0.1, z: 0 },
        orientation: { x: 0, y: 0, z: 0, w: 1, yaw: 0.1 },
      },
    ],
  },
  motion_status: {
    header: { stamp: { sec: 0, nanosec: 0 }, frame_id: "map" },
    command_id: 21,
    active: true,
    goal_reached: false,
    obstacle_detected: true,
    current_pose: {
      header: { stamp: { sec: 0, nanosec: 0 }, frame_id: "map" },
      position: { x: 0, y: 0, z: 0 },
      orientation: { x: 0, y: 0, z: 0, w: 1, yaw: 0 },
    },
    remaining_distance: 3.2,
    heading_error: -0.12,
  },
  obstacle_report: {
    header: { stamp: { sec: 0, nanosec: 0 }, frame_id: "map" },
    active: true,
    is_dynamic: true,
    blocks_path: true,
    severity: 3,
    distance: 0.92,
    bearing: 0.28,
    obstacle_point: { x: 0.9, y: 0.25, z: 0 },
    source: "scan",
  },
};
