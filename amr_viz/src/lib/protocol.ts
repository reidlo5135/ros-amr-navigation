export type Header = {
  stamp: {
    sec: number;
    nanosec: number;
  };
  frame_id: string;
};

export type Vector3 = {
  x: number;
  y: number;
  z: number;
};

export type Pose = {
  header: Header;
  position: Vector3;
  orientation: {
    x: number;
    y: number;
    z: number;
    w: number;
    yaw: number;
  };
};

export type PathMessage = {
  header: Header;
  poses: Pose[];
};

export type OccupancyGridMessage = {
  header: Header;
  info: {
    width: number;
    height: number;
    resolution: number;
    origin: {
      position: Vector3;
      orientation: {
        x: number;
        y: number;
        z: number;
        w: number;
        yaw: number;
      };
    };
  };
  data: number[];
};

export type MotionStatusMessage = {
  header: Header;
  command_id: number;
  active: boolean;
  goal_reached: boolean;
  obstacle_detected: boolean;
  current_pose: Pose;
  remaining_distance: number;
  heading_error: number;
};

export type LaserScanMessage = {
  header: Header;
  angle_min: number;
  angle_max: number;
  angle_increment: number;
  range_min: number;
  range_max: number;
  ranges: number[];
};

export type TransformMessage = {
  header: Header;
  child_frame_id: string;
  translation: Vector3;
  rotation: {
    x: number;
    y: number;
    z: number;
    w: number;
    yaw: number;
  };
};

export type TfMessage = {
  transforms: TransformMessage[];
};

export type RobotDescriptionMessage = {
  data: string;
  footprint_polygon?: number[];
};

export type BridgeState = {
  robot_pose?: Pose;
  global_path?: PathMessage;
  local_path?: PathMessage;
  map?: OccupancyGridMessage;
  global_costmap?: OccupancyGridMessage;
  local_costmap?: OccupancyGridMessage;
  motion_status?: MotionStatusMessage;
  scan?: LaserScanMessage;
  tf?: TfMessage;
  tf_static?: TfMessage;
  robot_description?: RobotDescriptionMessage;
};

export type BridgeEnvelope =
  | {
      type: "hello";
      payload: {
        server: string;
        protocol_version: number;
        capabilities: string[];
      };
    }
  | {
      type: "snapshot";
      payload: BridgeState;
    }
  | {
      type: "topic_update";
      channel: keyof BridgeState;
      payload: BridgeState[keyof BridgeState];
    }
  | {
      type: "command_result" | "command_feedback";
      channel: string;
      payload: {
        id: string;
        success: boolean;
        message: string;
        data?: unknown;
      };
    }
  | {
      type: "error";
      payload: {
        message: string;
      };
    }
  | {
      type: "pong";
      payload: {
        id: string;
      };
    };

export type BridgeCommand =
  | {
      type: "command";
      id: string;
      command: "navigate_to_pose";
      payload: {
        x: number;
        y: number;
        yaw: number;
        frame_id: string;
      };
    }
  | {
      type: "command";
      id: string;
      command: "set_initial_pose";
      payload: {
        x: number;
        y: number;
        yaw: number;
        frame_id: string;
        covariance_x: number;
        covariance_y: number;
        covariance_yaw: number;
      };
    };
