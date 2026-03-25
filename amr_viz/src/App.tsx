import { startTransition, useEffect, useRef, useState } from "react";

import { SceneViewport } from "./components/SceneViewport";
import type { BridgeState, TfMessage, TransformMessage } from "./lib/protocol";
import { VizMqttClient, type VizMqttMessage } from "./lib/mqtt";

function createCommandId() {
  return `${Date.now()}-${Math.round(Math.random() * 10000)}`;
}

function defaultMqttUrl() {
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

function createInitialState() {
  return {};
}

const telemetryTopicMap: Record<string, keyof BridgeState> = {
  "amr/robot/turtlebot3/viz/robot_pose": "robot_pose",
  "amr/robot/turtlebot3/viz/global_path": "global_path",
  "amr/robot/turtlebot3/viz/local_path": "local_path",
  "amr/robot/turtlebot3/viz/map": "map",
  "amr/robot/turtlebot3/viz/global_costmap": "global_costmap",
  "amr/robot/turtlebot3/viz/local_costmap": "local_costmap",
  "amr/robot/turtlebot3/viz/motion_status": "motion_status",
  "amr/robot/turtlebot3/viz/scan": "scan",
  "amr/robot/turtlebot3/viz/tf": "tf",
  "amr/robot/turtlebot3/viz/tf_static": "tf_static",
  "amr/robot/turtlebot3/viz/robot_description": "robot_description",
};

const topicSubscriptions = [
  "amr/robot/turtlebot3/viz/#",
  "amr/response/#",
  "amr/feedback/#",
  "amr/status/#",
];

function isObjectPayload(value: unknown): value is Record<string, unknown> {
  return typeof value === "object" && value !== null;
}

function isTfMessage(value: unknown): value is TfMessage {
  return isObjectPayload(value) && Array.isArray(value.transforms);
}

function mergeTfMessages(current: TfMessage | undefined, incoming: TfMessage): TfMessage {
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

type LayerVisibility = {
  grid: boolean;
  map: boolean;
  globalCostmap: boolean;
  localCostmap: boolean;
  footprint: boolean;
  blockedDebug: boolean;
  collisionDebug: boolean;
  robot: boolean;
  paths: boolean;
  scan: boolean;
  tf: boolean;
};

type GoalMarker = {
  x: number;
  y: number;
  yaw: number;
  kind: "goal" | "initial_pose";
};

type InteractionMode = "idle" | "goal" | "initial_pose";
type GoalLifecycleState =
  | "Idle"
  | "Pending"
  | "Running"
  | "Recovering"
  | "Canceling"
  | "Canceled"
  | "Aborted"
  | "Reached"
  | "Rejected";

export default function App() {
  const clientRef = useRef(new VizMqttClient());
  const binaryTopicsRef = useRef(new Set<string>());
  const pendingTelemetryRef = useRef<Partial<BridgeState>>({});
  const flushFrameRef = useRef<number | null>(null);
  const [mqttUrl, setMqttUrl] = useState(defaultMqttUrl);
  const [bridgeState, setBridgeState] = useState<BridgeState>(createInitialState);
  const [connectionLabel, setConnectionLabel] = useState("Disconnected");
  const [events, setEvents] = useState<string[]>([
    "AMR Viz MQTT client ready",
    "Connect to the broker WebSocket endpoint",
  ]);
  const [goalX, setGoalX] = useState("2.5");
  const [goalY, setGoalY] = useState("0.0");
  const [goalYaw, setGoalYaw] = useState("0.0");
  const [goalMarker, setGoalMarker] = useState<GoalMarker | null>(null);
  const [goalLifecycle, setGoalLifecycle] = useState<GoalLifecycleState>("Idle");
  const [interactionMode, setInteractionMode] = useState<InteractionMode>("idle");
  const [layerVisibility, setLayerVisibility] = useState<LayerVisibility>({
    grid: true,
    map: true,
    globalCostmap: true,
    localCostmap: true,
    footprint: true,
    blockedDebug: true,
    collisionDebug: true,
    robot: true,
    paths: true,
    scan: true,
    tf: true,
  });

  useEffect(() => {
    const client = clientRef.current;
    const flushTelemetry = () => {
      flushFrameRef.current = null;
      const pending = pendingTelemetryRef.current;
      pendingTelemetryRef.current = {};
      if (Object.keys(pending).length === 0) {
        return;
      }
      setBridgeState((current) => ({
        ...current,
        ...pending,
      }));
    };

    const unsubscribeStatus = client.onStatusChange((status, detail) => {
      startTransition(() => {
        if (status === "connecting") {
          setConnectionLabel(`Connecting: ${detail ?? mqttUrl}`);
          setEvents((current) => [`Connecting: ${detail ?? mqttUrl}`, ...current].slice(0, 10));
          return;
        }

        if (status === "connected") {
          setConnectionLabel(`MQTT connected: ${detail ?? mqttUrl}`);
          setEvents((current) => [`MQTT connected: ${detail ?? mqttUrl}`, ...current].slice(0, 10));
          return;
        }

        if (status === "error") {
          setConnectionLabel(`MQTT error: ${detail ?? mqttUrl}`);
          setEvents((current) => [`MQTT error: ${detail ?? mqttUrl}`, ...current].slice(0, 10));
          return;
        }

        setConnectionLabel("Disconnected");
      });
    });

    const unsubscribeMessage = client.onMessage((message: VizMqttMessage) => {
      const mappedChannel = telemetryTopicMap[message.topic];
      if (mappedChannel && isObjectPayload(message.json)) {
        if ((mappedChannel === "tf" || mappedChannel === "tf_static") && isTfMessage(message.json)) {
          const currentValue = (pendingTelemetryRef.current[mappedChannel] as TfMessage | undefined) ??
            (bridgeState[mappedChannel] as TfMessage | undefined);
          pendingTelemetryRef.current[mappedChannel] = mergeTfMessages(currentValue, message.json);
        } else {
          pendingTelemetryRef.current[mappedChannel] = message.json as BridgeState[keyof BridgeState];
        }
        if (flushFrameRef.current === null) {
          flushFrameRef.current = window.requestAnimationFrame(flushTelemetry);
        }
        return;
      }

      if (mappedChannel && !message.text) {
        if (!binaryTopicsRef.current.has(message.topic)) {
          binaryTopicsRef.current.add(message.topic);
          setEvents((current) => [
            `Binary telemetry on ${message.topic}; waiting for modeled viz topics`,
            ...current,
          ].slice(0, 10));
        }
        return;
      }

      if (message.topic.startsWith("amr/response/") && isObjectPayload(message.json)) {
        const success = message.json.success === true ? "OK" : "FAIL";
        const detail = typeof message.json.message === "string" ? message.json.message : "ack";
        if (message.topic === "amr/response/navigate_to_pose") {
          const accepted = message.json.accepted === true;
          const completed = message.json.completed === true;
          const succeeded = message.json.success === true;
          const responseMessage =
            typeof message.json.message === "string" ? message.json.message.toLowerCase() : "";

          if (accepted && !completed) {
            setGoalLifecycle("Running");
          } else if (completed) {
            if (succeeded) {
              setGoalLifecycle("Reached");
            } else if (responseMessage.includes("cancel")) {
              setGoalLifecycle("Canceled");
            } else {
              setGoalLifecycle("Aborted");
            }
          } else if (!succeeded) {
            setGoalLifecycle("Rejected");
          }
        }
        setEvents((current) => [`${message.topic}: ${success} ${detail}`, ...current].slice(0, 10));
        return;
      }

      if ((message.topic.startsWith("amr/feedback/") || message.topic.startsWith("amr/status/")) && !message.text) {
        if (!binaryTopicsRef.current.has(message.topic)) {
          binaryTopicsRef.current.add(message.topic);
          setEvents((current) => [
            `Binary action stream on ${message.topic}`,
            ...current,
          ].slice(0, 10));
        }
        return;
      }

      if (message.text) {
        setEvents((current) => [`${message.topic}: ${message.text}`, ...current].slice(0, 10));
      }
    });

    return () => {
      if (flushFrameRef.current !== null) {
        window.cancelAnimationFrame(flushFrameRef.current);
        flushFrameRef.current = null;
      }
      unsubscribeStatus();
      unsubscribeMessage();
    };
  }, [bridgeState, mqttUrl]);

  const connect = () => {
    clientRef.current.connect(mqttUrl, topicSubscriptions);
  };

  const disconnect = () => {
    clientRef.current.disconnect();
    setConnectionLabel("Disconnected");
  };

  const publishJson = (topic: string, payload: Record<string, unknown>) => {
    const sent = clientRef.current.publishJson(topic, payload, { qos: 0, retain: false });
    if (!sent) {
      setEvents((current) => ["MQTT client is not connected", ...current].slice(0, 10));
    }
  };

  const toggleLayer = (layer: keyof LayerVisibility) => {
    setLayerVisibility((current) => ({
      ...current,
      [layer]: !current[layer],
    }));
  };

  const sendGoal = (x: number, y: number, yaw: number) => {
    setInteractionMode("idle");
    setGoalMarker({ x, y, yaw, kind: "goal" });
    setGoalLifecycle("Pending");
    setGoalX(x.toFixed(2));
    setGoalY(y.toFixed(2));
    setGoalYaw(yaw.toFixed(2));
    publishJson("amr/command/navigate_to_pose", {
      request_id: createCommandId(),
      goal_pose: {
        header: {
          stamp: { sec: 0, nanosec: 0 },
          frame_id: "map",
        },
        pose: {
          position: { x, y, z: 0.0 },
          orientation: {
            x: 0.0,
            y: 0.0,
            z: Math.sin(yaw * 0.5),
            w: Math.cos(yaw * 0.5),
          },
        },
      },
    });
  };

  const setInitialPose = (x: number, y: number, yaw: number) => {
    setInteractionMode("idle");
    setGoalMarker({ x, y, yaw, kind: "initial_pose" });
    setGoalX(x.toFixed(2));
    setGoalY(y.toFixed(2));
    setGoalYaw(yaw.toFixed(2));
    publishJson("amr/command/set_initial_pose", {
      request_id: createCommandId(),
      frame_id: "map",
      x,
      y,
      yaw,
      covariance_x: 0.25,
      covariance_y: 0.25,
      covariance_yaw: 0.06853891945200942,
    });
  };

  const handleScenePoseSelection = (x: number, y: number, yaw: number) => {
    setGoalX(x.toFixed(2));
    setGoalY(y.toFixed(2));
    setGoalYaw(yaw.toFixed(2));
  };

  const handleScenePosePlacement = (
    mode: Exclude<InteractionMode, "idle">,
    x: number,
    y: number,
    yaw: number,
  ) => {
    setGoalX(x.toFixed(2));
    setGoalY(y.toFixed(2));
    setGoalYaw(yaw.toFixed(2));
    if (mode === "goal") {
      sendGoal(x, y, yaw);
      return;
    }
    setInitialPose(x, y, yaw);
  };

  const toggleGoalMode = () => {
    if (interactionMode === "goal") {
      sendGoal(Number(goalX), Number(goalY), Number(goalYaw));
      return;
    }
    setInteractionMode("goal");
  };

  const toggleInitialPoseMode = () => {
    if (interactionMode === "initial_pose") {
      setInitialPose(Number(goalX), Number(goalY), Number(goalYaw));
      return;
    }
    setInteractionMode("initial_pose");
  };

  const resolvedGoalLifecycle: GoalLifecycleState = (() => {
    if (
      goalLifecycle === "Aborted" ||
      goalLifecycle === "Canceled" ||
      goalLifecycle === "Reached" ||
      goalLifecycle === "Rejected")
    {
      return goalLifecycle;
    }

    const motionStatus = bridgeState.motion_status;
    if (!motionStatus) {
      return goalLifecycle;
    }

    if (motionStatus.goal_reached) {
      return "Reached";
    }

    if (goalLifecycle === "Canceling") {
      return "Canceling";
    }

    if (motionStatus.active) {
      if (motionStatus.blocked || motionStatus.stalled || !motionStatus.local_plan_valid) {
        return "Recovering";
      }
      return "Running";
    }

    return goalLifecycle;
  })();

  return (
    <main className="app-shell">
      <header className="topbar">
        <div className="topbar-title">AMR Viz</div>
        <div className="topbar-status">
          <span className="topbar-chip">{connectionLabel}</span>
          <span className="topbar-chip">
            Pose{" "}
            {bridgeState.robot_pose
              ? `${bridgeState.robot_pose.position.x.toFixed(2)}, ${bridgeState.robot_pose.position.y.toFixed(2)}`
              : "--"}
          </span>
        </div>
      </header>

      <section className="workspace">
        <aside className="sidebar sidebar-left">
          <section className="panel-card">
            <div className="panel-section-title">MQTT</div>
            <label className="field-label">
              <span>Broker WS</span>
              <input value={mqttUrl} onChange={(event) => setMqttUrl(event.target.value)} />
            </label>
            <div className="button-row">
              <button onClick={connect}>Connect</button>
              <button className="secondary" onClick={disconnect}>
                Disconnect
              </button>
            </div>
          </section>

          <section className="panel-card">
            <div className="panel-section-title">Goal Control</div>
            <div className="field-grid field-grid-triple">
              <label className="field-label">
                <span>X</span>
                <input value={goalX} onChange={(event) => setGoalX(event.target.value)} />
              </label>
              <label className="field-label">
                <span>Y</span>
                <input value={goalY} onChange={(event) => setGoalY(event.target.value)} />
              </label>
              <label className="field-label">
                <span>Yaw</span>
                <input value={goalYaw} onChange={(event) => setGoalYaw(event.target.value)} />
              </label>
            </div>
            <div className="button-row">
              <button
                className={interactionMode === "goal" ? "active-mode" : undefined}
                onClick={toggleGoalMode}
              >
                Send
              </button>
              <button
                className="danger"
                onClick={() => {
                  setInteractionMode("idle");
                  setGoalMarker(null);
                  setGoalLifecycle("Canceling");
                  publishJson("amr/command/cancel_navigate_to_pose", {
                    request_id: createCommandId(),
                  });
                }}
              >
                Cancel
              </button>
            </div>
            <div className="button-row single-row">
              <button
                className={`secondary${interactionMode === "initial_pose" ? " active-mode" : ""}`}
                onClick={toggleInitialPoseMode}
              >
                Set Initial Pose
              </button>
            </div>
            <div className="mode-hint">
              Select `Send` or `Set Initial Pose`, then left-drag on map to apply. Map interaction also updates
              X / Y / Yaw.
            </div>
          </section>

          <section className="panel-card">
            <div className="panel-section-title">Layers</div>
            <div className="layer-list">
              {[
                ["grid", "Grid"],
                ["map", "Raw SLAM Map"],
                ["globalCostmap", "Global Costmap"],
                ["localCostmap", "Local Costmap"],
                ["footprint", "Exact Footprint"],
                ["blockedDebug", "Blocked Footprint"],
                ["collisionDebug", "Collision Debug"],
                ["robot", "Robot"],
                ["paths", "Plans"],
                ["scan", "LaserScan"],
                ["tf", "TF"],
              ].map(([key, label]) => {
                const layerKey = key as keyof LayerVisibility;
                return (
                  <label className="layer-item" key={key}>
                    <input
                      type="checkbox"
                      checked={layerVisibility[layerKey]}
                      onChange={() => toggleLayer(layerKey)}
                    />
                    <span>{label}</span>
                  </label>
                );
              })}
            </div>
          </section>
        </aside>

        <section className="scene-panel">
          <div className="scene-toolbar">
            <span className="toolbar-label">3D View</span>
            <span className="toolbar-value">
              Map {bridgeState.map ? `${bridgeState.map.info.width}x${bridgeState.map.info.height}` : "--"}
            </span>
            <span className="toolbar-value">
              Global {bridgeState.global_path?.poses.length ?? 0} pts
            </span>
            <span className="toolbar-value">
              Local {bridgeState.local_path?.poses.length ?? 0} pts
            </span>
            <span className="toolbar-value">
              Scan {bridgeState.scan?.ranges.length ?? 0} rays
            </span>
            <span className="toolbar-value">
              TF {(bridgeState.tf?.transforms.length ?? 0) + (bridgeState.tf_static?.transforms.length ?? 0)} frames
            </span>
          </div>
          <SceneViewport
            state={bridgeState}
            layerVisibility={layerVisibility}
            goalMarker={goalMarker}
            onPoseSelection={handleScenePoseSelection}
            interactionMode={interactionMode}
            onPosePlacement={handleScenePosePlacement}
          />
        </section>

        <aside className="sidebar sidebar-right">
          <section className="panel-card">
            <div className="panel-section-title">Navigation</div>
            <div className="metric-list">
              <div className="metric-row">
                <span>Motion</span>
                <strong>{bridgeState.motion_status?.active ? "Navigating" : "Idle"}</strong>
              </div>
              <div className="metric-row">
                <span>Remaining</span>
                <strong>
                  {bridgeState.motion_status?.remaining_distance?.toFixed(2) ?? "--"} m
                </strong>
              </div>
              <div className="metric-row">
                <span>Heading</span>
                <strong>
                  {bridgeState.motion_status?.heading_error?.toFixed(2) ?? "--"} rad
                </strong>
              </div>
              <div className="metric-row">
                <span>Goal</span>
                <strong>{resolvedGoalLifecycle}</strong>
              </div>
              <div className="metric-row">
                <span>Blocked Source</span>
                <strong>
                  {bridgeState.motion_status?.costmap_blocked ? "Costmap" :
                    bridgeState.motion_status?.safety_gate_blocked ? "Safety Gate" :
                    bridgeState.motion_status?.blocked ? "Blocked" : "Clear"}
                </strong>
              </div>
            </div>
          </section>

          <section className="panel-card">
            <div className="panel-section-title">Events</div>
            <div className="event-log">
              {events.map((event) => (
                <div key={event} className="event-item">
                  {event}
                </div>
              ))}
            </div>
          </section>
        </aside>
      </section>
    </main>
  );
}
