export type LayerVisibility = {
  grid: boolean;
  map: boolean;
  globalCostmap: boolean;
  localCostmap: boolean;
  footprint: boolean;
  robot: boolean;
  globalPlan: boolean;
  localPlan: boolean;
  scan: boolean;
  tf: boolean;
};

export type GoalMarker = {
  x: number;
  y: number;
  yaw: number;
  kind: "goal" | "initial_pose";
};

export type InteractionMode = "idle" | "goal" | "initial_pose";

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
