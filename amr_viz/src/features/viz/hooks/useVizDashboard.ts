import { startTransition, useEffect, useRef, useState } from "react";

import { VizMqttClient, type VizMqttMessage } from "../../../lib/mqtt";
import type { BridgeState, TfMessage } from "../../../lib/protocol";
import {
  createCommandId,
  createInitialState,
  defaultMqttUrl,
  initialLayerVisibility,
  isObjectPayload,
  isTfMessage,
  mergeTfMessages,
  telemetryTopicMap,
  topicSubscriptions,
} from "../constants";
import type {
  GoalLifecycleState,
  GoalMarker,
  InteractionMode,
  LayerVisibility,
} from "../types";

export function useVizDashboard() {
  const clientRef = useRef(new VizMqttClient());
  const binaryTopicsRef = useRef(new Set<string>());
  const pendingTelemetryRef = useRef<Partial<BridgeState>>({});
  const flushFrameRef = useRef<number | null>(null);
  const bridgeStateRef = useRef<BridgeState>(createInitialState());

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
  const [layerVisibility, setLayerVisibility] = useState<LayerVisibility>(initialLayerVisibility);

  useEffect(() => {
    bridgeStateRef.current = bridgeState;
  }, [bridgeState]);

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
            (bridgeStateRef.current[mappedChannel] as TfMessage | undefined);
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
          setEvents((current) => [`Binary action stream on ${message.topic}`, ...current].slice(0, 10));
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

  const cancelGoal = () => {
    setInteractionMode("idle");
    setGoalMarker(null);
    setGoalLifecycle("Canceling");
    publishJson("amr/command/cancel_navigate_to_pose", {
      request_id: createCommandId(),
    });
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

  const batteryPercentage = (() => {
    const percentage = bridgeState.battery_state?.percentage;
    if (typeof percentage !== "number" || !Number.isFinite(percentage) || percentage < 0) {
      return null;
    }
    return Math.max(0, Math.min(100, percentage));
  })();

  return {
    mqttUrl,
    setMqttUrl,
    bridgeState,
    connectionLabel,
    events,
    goalX,
    setGoalX,
    goalY,
    setGoalY,
    goalYaw,
    setGoalYaw,
    goalMarker,
    interactionMode,
    layerVisibility,
    resolvedGoalLifecycle,
    batteryPercentage,
    connect,
    disconnect,
    toggleLayer,
    toggleGoalMode,
    toggleInitialPoseMode,
    cancelGoal,
    handleScenePoseSelection,
    handleScenePosePlacement,
  };
}
