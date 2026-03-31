import type { BridgeState, TfMessage, TransformMessage } from "../../lib/protocol";
import type { LayerVisibility, ViewMode } from "./types";

export function defaultRobotId() {
  const configuredRobotId = import.meta.env.VITE_AMR_VIZ_ROBOT_ID as string | undefined;
  if (configuredRobotId && configuredRobotId.trim().length > 0) {
    return configuredRobotId.trim();
  }

  return "turtlebot3";
}

export function normalizeRobotId(value: string) {
  const trimmed = value.trim();
  if (trimmed.length === 0) {
    return defaultRobotId();
  }

  return trimmed;
}

export function mqttTopicRoot(robotId: string) {
  return `/amr/${normalizeRobotId(robotId)}`;
}

export function buildTelemetryTopicMap(robotId: string): Record<string, keyof BridgeState> {
  const root = mqttTopicRoot(robotId);
  return {
    [`${root}/viz/robot_pose`]: "robot_pose",
    [`${root}/viz/mapping_pose`]: "mapping_pose",
    [`${root}/viz/global_path`]: "global_path",
    [`${root}/viz/local_path`]: "local_path",
    [`${root}/viz/map`]: "map",
    [`${root}/viz/temp_map/raw`]: "temp_map_raw",
    [`${root}/viz/temp_map/refined`]: "temp_map_refined",
    [`${root}/viz/global_costmap`]: "global_costmap",
    [`${root}/viz/local_costmap`]: "local_costmap",
    [`${root}/viz/motion_status`]: "motion_status",
    [`${root}/viz/scan`]: "scan",
    [`${root}/viz/tf`]: "tf",
    [`${root}/viz/tf_static`]: "tf_static",
    [`${root}/viz/robot_description`]: "robot_description",
    [`${root}/viz/battery_state`]: "battery_state",
    [`${root}/viz/slam_graph`]: "slam_graph",
  };
}

export function buildTopicSubscriptions(robotId: string) {
  const root = mqttTopicRoot(robotId);
  return [
    `${root}/viz/#`,
    `${root}/response/#`,
    `${root}/feedback/#`,
    `${root}/status/#`,
  ];
}

export function buildCommandTopic(robotId: string, commandName: string) {
  return `${mqttTopicRoot(robotId)}/command/${commandName}`;
}

export const initialLayerVisibility: LayerVisibility = {
  grid: true,
  map: true,
  rawMap: false,
  refinedMap: false,
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
  rawMap: true,
  refinedMap: true,
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
