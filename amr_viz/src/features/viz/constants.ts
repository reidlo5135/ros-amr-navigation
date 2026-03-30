import type { BridgeState, TfMessage, TransformMessage } from "../../lib/protocol";
import type { LayerVisibility, ViewMode } from "./types";

export const telemetryTopicMap: Record<string, keyof BridgeState> = {
  "amr/robot/turtlebot3/viz/robot_pose": "robot_pose",
  "amr/robot/turtlebot3/viz/mapping_pose": "mapping_pose",
  "amr/robot/turtlebot3/viz/global_path": "global_path",
  "amr/robot/turtlebot3/viz/local_path": "local_path",
  "amr/robot/turtlebot3/viz/map": "map",
  "amr/robot/turtlebot3/viz/temp_map": "temp_map",
  "amr/robot/turtlebot3/viz/global_costmap": "global_costmap",
  "amr/robot/turtlebot3/viz/local_costmap": "local_costmap",
  "amr/robot/turtlebot3/viz/motion_status": "motion_status",
  "amr/robot/turtlebot3/viz/scan": "scan",
  "amr/robot/turtlebot3/viz/tf": "tf",
  "amr/robot/turtlebot3/viz/tf_static": "tf_static",
  "amr/robot/turtlebot3/viz/robot_description": "robot_description",
  "amr/robot/turtlebot3/viz/battery_state": "battery_state",
  "amr/robot/turtlebot3/viz/slam_graph": "slam_graph",
};

export const topicSubscriptions = [
  "amr/robot/turtlebot3/viz/#",
  "amr/response/#",
  "amr/feedback/#",
  "amr/status/#",
];

export const initialLayerVisibility: LayerVisibility = {
  grid: true,
  map: true,
  tempMap: false,
  globalCostmap: true,
  localCostmap: true,
  footprint: true,
  robot: true,
  globalPlan: true,
  localPlan: true,
  scan: true,
  tf: true,
  keyframes: false,
  graphEdges: false,
  loopMarkers: false,
};

export const mappingLayerVisibility: LayerVisibility = {
  grid: true,
  map: false,
  tempMap: true,
  globalCostmap: false,
  localCostmap: false,
  footprint: true,
  robot: true,
  globalPlan: false,
  localPlan: false,
  scan: true,
  tf: true,
  keyframes: true,
  graphEdges: true,
  loopMarkers: true,
};

export function layerProfileForMode(mode: ViewMode): LayerVisibility {
  return mode === "mapping" ? mappingLayerVisibility : initialLayerVisibility;
}

export function createCommandId() {
  return `${Date.now()}-${Math.round(Math.random() * 10000)}`;
}

export function defaultMqttUrl() {
  const configuredUrl = import.meta.env.VITE_AMR_VIZ_MQTT_URL as string | undefined;
  if (configuredUrl) {
    return configuredUrl;
  }

  if (typeof window !== "undefined") {
    const protocol = window.location.protocol === "https:" ? "wss:" : "ws:";
    const host = window.location.hostname || "127.0.0.1";
    return `${protocol}//${host}:9001/mqtt`;
  }

  return "ws://127.0.0.1:9001/mqtt";
}

export function createInitialState() {
  return {};
}

export function isObjectPayload(value: unknown): value is Record<string, unknown> {
  return typeof value === "object" && value !== null;
}

export function isTfMessage(value: unknown): value is TfMessage {
  return isObjectPayload(value) && Array.isArray(value.transforms);
}

export function mergeTfMessages(current: TfMessage | undefined, incoming: TfMessage): TfMessage {
  const byChildFrame = new Map<string, TransformMessage>();
  for (const transform of current?.transforms ?? []) {
    byChildFrame.set(transform.child_frame_id, transform);
  }
  for (const transform of incoming.transforms) {
    byChildFrame.set(transform.child_frame_id, transform);
  }
  return {
    transforms: Array.from(byChildFrame.values()),
  };
}
