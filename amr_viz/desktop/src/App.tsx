import { startTransition, useEffect, useRef, useState } from "react";

import { SceneViewport } from "./components/SceneViewport";
import { mockState } from "./lib/mock-state";
import type { BridgeCommand, BridgeEnvelope, BridgeState } from "./lib/protocol";
import { VizSocketClient } from "./lib/socket";

function createCommandId() {
  return `${Date.now()}-${Math.round(Math.random() * 10000)}`;
}

function defaultBridgeUrl() {
  const configuredUrl = import.meta.env.VITE_AMR_VIZ_BRIDGE_URL as string | undefined;
  if (configuredUrl) {
    return configuredUrl;
  }

  if (typeof window !== "undefined") {
    const pageProtocol = window.location.protocol === "https:" ? "wss:" : "ws:";
    const pageHost = window.location.hostname || "127.0.0.1";
    return `${pageProtocol}//${pageHost}:8765`;
  }

  return "ws://127.0.0.1:8765";
}

export default function App() {
  const clientRef = useRef(new VizSocketClient());
  const [bridgeUrl, setBridgeUrl] = useState(defaultBridgeUrl);
  const [bridgeState, setBridgeState] = useState<BridgeState>(mockState);
  const [connectionLabel, setConnectionLabel] = useState("Disconnected");
  const [events, setEvents] = useState<string[]>([
    "AMR Viz scaffold ready",
    "Connect the bridge to replace the mock scene",
  ]);
  const [goalX, setGoalX] = useState("2.5");
  const [goalY, setGoalY] = useState("0.0");
  const [goalYaw, setGoalYaw] = useState("0.0");

  useEffect(() => {
    const client = clientRef.current;
    const unsubscribeStatus = client.onStatusChange((status, detail) => {
      startTransition(() => {
        if (status === "connecting") {
          setConnectionLabel(`Connecting: ${detail ?? bridgeUrl}`);
          setEvents((current) => [`Connecting: ${detail ?? bridgeUrl}`, ...current].slice(0, 8));
          return;
        }

        if (status === "connected") {
          setConnectionLabel(`Socket connected: ${detail ?? bridgeUrl}`);
          setEvents((current) => [`Socket connected: ${detail ?? bridgeUrl}`, ...current].slice(0, 8));
          return;
        }

        if (status === "error") {
          setConnectionLabel(`Connection error: ${detail ?? bridgeUrl}`);
          setEvents((current) => [`Bridge connection error: ${detail ?? bridgeUrl}`, ...current].slice(0, 8));
          return;
        }

        setConnectionLabel("Disconnected");
      });
    });

    const unsubscribeMessage = client.onMessage((message: BridgeEnvelope) => {
      startTransition(() => {
        if (message.type === "hello") {
          setConnectionLabel(`Bridge ready: ${message.payload.server}`);
          setEvents((current) => [
            `Bridge protocol v${message.payload.protocol_version} ready`,
            ...current,
          ].slice(0, 8));
          return;
        }

        if (message.type === "snapshot") {
          setBridgeState((current) => ({ ...current, ...message.payload }));
          setEvents((current) => ["Snapshot received", ...current].slice(0, 8));
          return;
        }

        if (message.type === "topic_update") {
          setBridgeState((current) => ({
            ...current,
            [message.channel]: message.payload,
          }));
          return;
        }

        if (message.type === "command_result" || message.type === "command_feedback") {
          setEvents((current) => [
            `${message.channel}: ${message.payload.message}`,
            ...current,
          ].slice(0, 8));
          return;
        }

        if (message.type === "error") {
          setEvents((current) => [`Bridge error: ${message.payload.message}`, ...current].slice(0, 8));
        }
      });
    });

    return () => {
      unsubscribeStatus();
      unsubscribeMessage();
    };
  }, [bridgeUrl]);

  const connect = () => {
    const client = clientRef.current;
    client.connect(bridgeUrl);
  };

  const disconnect = () => {
    clientRef.current.disconnect();
    setConnectionLabel("Disconnected");
  };

  const sendCommand = (command: BridgeCommand) => {
    const sent = clientRef.current.send(command);
    if (!sent) {
      setEvents((current) => ["Bridge is not connected", ...current].slice(0, 8));
    }
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
            <div className="panel-section-title">Bridge</div>
            <label className="field-label">
              <span>WebSocket</span>
              <input
                value={bridgeUrl}
                onChange={(event) => setBridgeUrl(event.target.value)}
              />
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
                onClick={() =>
                  sendCommand({
                    type: "command",
                    id: createCommandId(),
                    command: "navigate_to_pose",
                    payload: {
                      x: Number(goalX),
                      y: Number(goalY),
                      yaw: Number(goalYaw),
                      frame_id: "map",
                    },
                  })
                }
              >
                Send Goal
              </button>
              <button
                className="secondary"
                onClick={() =>
                  sendCommand({
                    type: "command",
                    id: createCommandId(),
                    command: "set_initial_pose",
                    payload: {
                      x: Number(goalX),
                      y: Number(goalY),
                      yaw: Number(goalYaw),
                      frame_id: "map",
                      covariance_x: 0.25,
                      covariance_y: 0.25,
                      covariance_yaw: 0.06853891945200942,
                    },
                  })
                }
              >
                Set Initial Pose
              </button>
            </div>
          </section>
        </aside>

        <section className="scene-panel">
          <div className="scene-toolbar">
            <span className="toolbar-label">3D View</span>
            <span className="toolbar-value">
              Global {bridgeState.global_path?.poses.length ?? 0} pts
            </span>
            <span className="toolbar-value">
              Local {bridgeState.local_path?.poses.length ?? 0} pts
            </span>
          </div>
          <SceneViewport state={bridgeState} />
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
                    : "None"}
                </strong>
              </div>
              <div className="metric-row">
                <span>Blocks</span>
                <strong>{bridgeState.obstacle_report?.blocks_path ? "Yes" : "No"}</strong>
              </div>
              <div className="metric-row">
                <span>Distance</span>
                <strong>
                  {bridgeState.obstacle_report?.distance?.toFixed(2) ?? "--"} m
                </strong>
              </div>
              <div className="metric-row">
                <span>Bearing</span>
                <strong>
                  {bridgeState.obstacle_report?.bearing?.toFixed(2) ?? "--"} rad
                </strong>
              </div>
            </div>
          </section>

          <section className="panel-card log-panel">
            <div className="panel-section-title">Events</div>
            <div className="event-list">
              {events.map((event, index) => (
                <div className="event-item" key={`${event}-${index}`}>
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
