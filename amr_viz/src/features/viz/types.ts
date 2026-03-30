export type LayerVisibility = {
  grid: boolean;
  map: boolean;
  rawMap: boolean;
  refinedMap: boolean;
  globalCostmap: boolean;
  localCostmap: boolean;
  footprint: boolean;
  robot: boolean;
  globalPlan: boolean;
  localPlan: boolean;
  scan: boolean;
  tf: boolean;
  keyframes: boolean;
  graphEdges: boolean;
  loopMarkers: boolean;
};

export type GoalMarker = {
  x: number;
  y: number;
  yaw: number;
  kind: "goal" | "initial_pose";
};

export type InteractionMode = "idle" | "goal" | "initial_pose";

export type ViewMode = "nav" | "mapping";

export type GoalLifecycleState =
  | "Idle"
  | "Pending"
  | "Running"
  | "Recovering"
  | "Canceling"
  | "Canceled"
  | "Aborted"
  | "Reached"
  | "Rejected";
