import { startTransition, useEffect, useRef, useState } from "react";

import { SceneViewport } from "./components/SceneViewport";
import type { BridgeState } from "./lib/protocol";
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
  "amr/robot/turtlebot3/viz/obstacle_report": "obstacle_report",
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

type LayerVisibility = {
  grid: boolean;
  map: boolean;
  globalCostmap: boolean;
  localCostmap: boolean;
  robot: boolean;
  paths: boolean;
  scan: boolean;
  tf: boolean;
  obstacle: boolean;
};

type GoalMarker = {
  x: number;
  y: number;
  yaw: number;
  kind: "goal" | "initial_pose";
};

type InteractionMode = "idle" | "goal" | "initial_pose";

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
  const [interactionMode, setInteractionMode] = useState<InteractionMode>("idle");
  const [layerVisibility, setLayerVisibility] = useState<LayerVisibility>({
    grid: true,
    map: true,
    globalCostmap: true,
    localCostmap: true,
    robot: true,
    paths: true,
    scan: true,
    tf: true,
    obstacle: true,
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
        pendingTelemetryRef.current[mappedChannel] = message.json as BridgeState[keyof BridgeState];
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
  }, [mqttUrl]);

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
    setGoalMarker({ x, y, yaw, kind: "goal" });
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

  const handleScenePosePlacement = (mode: Exclude<InteractionMode, "idle">, x: number, y: number, yaw: number) => {
    if (mode === "goal") {
      sendGoal(x, y, yaw);
    } else {
      setInitialPose(x, y, yaw);
    }
    setInteractionMode("idle");
  };

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
          <span className="topbar-chip">
            Obstacle{" "}
            {bridgeState.obstacle_report?.active
              ? `${bridgeState.obstacle_report.distance.toFixed(2)} m`
              : "clear"}
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
            <div className="button-stack compact-stack">
              <button onClick={connect}>Connect</button>
              <button className="secondary" onClick={disconnect}>
                Disconnect
              </button>
            </div>
          </section>

          <section className="panel-card">
            <div className="panel-section-title">Quick Goal</div>
            <div className="field-grid">
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
            <div className="button-stack">
              <button
                onClick={() => sendGoal(Number(goalX), Number(goalY), Number(goalYaw))}
              >
                Send Goal
              </button>
              <button
                className={interactionMode === "goal" ? "secondary active-mode" : "secondary"}
                onClick={() =>
                  setInteractionMode((current) => (current === "goal" ? "idle" : "goal"))
                }
              >
                Goal On Map
              </button>
              <button
                className="danger"
                onClick={() => {
                  setGoalMarker(null);
                  setInteractionMode("idle");
                  publishJson("amr/command/cancel_navigate_to_pose", {
                    request_id: createCommandId(),
                  });
                }}
              >
                Cancel Goal
              </button>
              <button
                className="secondary"
                onClick={() => setInitialPose(Number(goalX), Number(goalY), Number(goalYaw))}
              >
                Set Initial Pose
              </button>
              <button
                className={interactionMode === "initial_pose" ? "secondary active-mode" : "secondary"}
                onClick={() =>
                  setInteractionMode((current) => (current === "initial_pose" ? "idle" : "initial_pose"))
                }
              >
                Initial Pose On Map
              </button>
            </div>
            {interactionMode !== "idle" ? (
              <div className="mode-hint">
                {interactionMode === "goal"
                  ? "Drag on map to set goal heading"
                  : "Drag on map to set initial pose heading"}
              </div>
            ) : null}
          </section>

          <section className="panel-card">
            <div className="panel-section-title">Layers</div>
            <div className="layer-list">
              {[
                ["grid", "Grid"],
                ["map", "Raw SLAM Map"],
                ["globalCostmap", "Global Costmap"],
                ["localCostmap", "Local Costmap"],
                ["robot", "Robot"],
                ["paths", "Plans"],
                ["scan", "LaserScan"],
                ["tf", "TF"],
                ["obstacle", "Obstacle"],
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
                <strong>{bridgeState.motion_status?.goal_reached ? "Reached" : "Running"}</strong>
              </div>
            </div>
          </section>

          <section className="panel-card">
            <div className="panel-section-title">Obstacle</div>
            <div className="metric-list">
              <div className="metric-row">
                <span>Type</span>
                <strong>
                  {bridgeState.obstacle_report?.active
                    ? bridgeState.obstacle_report.is_dynamic
                      ? "Dynamic"
                      : "Static"
                    : "Clear"}
                </strong>
              </div>
              <div className="metric-row">
                <span>Distance</span>
                <strong>
                  {bridgeState.obstacle_report?.distance?.toFixed(2) ?? "--"} m
                </strong>
              </div>
              <div className="metric-row">
                <span>Blocks Path</span>
                <strong>{bridgeState.obstacle_report?.blocks_path ? "Yes" : "No"}</strong>
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
