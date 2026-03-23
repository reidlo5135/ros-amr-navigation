import mqtt, { type IClientOptions, type MqttClient } from "mqtt";

type StatusListener = (
  status: "connecting" | "connected" | "disconnected" | "error",
  detail?: string,
) => void;

export type VizMqttMessage = {
  topic: string;
  payload: Uint8Array;
  text?: string;
  json?: unknown;
};

type MessageListener = (message: VizMqttMessage) => void;

type PublishOptions = {
  qos?: 0 | 1 | 2;
  retain?: boolean;
};

function createClientId() {
  return `amr-viz-${Math.random().toString(16).slice(2, 10)}`;
}

function buildCandidateUrls(url: string) {
  const candidates = [url];

  if (url.endsWith("/mqtt")) {
    candidates.push(url.slice(0, -5));
  } else if (/^wss?:\/\/[^/]+$/i.test(url)) {
    candidates.push(`${url}/mqtt`);
  }

  return Array.from(new Set(candidates.filter((candidate) => candidate.length > 0)));
}

export class VizMqttClient {
  private client?: MqttClient;
  private listeners = new Set<MessageListener>();
  private statusListeners = new Set<StatusListener>();
  private connectAttempt = 0;
  private fallbackTimer?: ReturnType<typeof setTimeout>;

  connect(url: string, subscriptions: string[]) {
    this.disconnect();
    const candidates = buildCandidateUrls(url);
    const attemptId = ++this.connectAttempt;
    this.tryConnect(candidates, 0, subscriptions, attemptId);
  }

  disconnect() {
    this.connectAttempt += 1;
    if (this.fallbackTimer) {
      clearTimeout(this.fallbackTimer);
      this.fallbackTimer = undefined;
    }
    if (this.client) {
      this.client.end(true);
      this.client = undefined;
    }
    this.emitStatus("disconnected");
  }

  publishJson(topic: string, payload: Record<string, unknown>, options?: PublishOptions) {
    if (!this.client || !this.client.connected) {
      return false;
    }

    this.client.publish(topic, JSON.stringify(payload), {
      qos: options?.qos ?? 0,
      retain: options?.retain ?? false,
    });
    return true;
  }

  onMessage(listener: MessageListener) {
    this.listeners.add(listener);
    return () => this.listeners.delete(listener);
  }

  onStatusChange(listener: StatusListener) {
    this.statusListeners.add(listener);
    return () => this.statusListeners.delete(listener);
  }

  private emitStatus(
    status: "connecting" | "connected" | "disconnected" | "error",
    detail?: string,
  ) {
    for (const listener of this.statusListeners) {
      listener(status, detail);
    }
  }

  private tryConnect(
    candidates: string[],
    index: number,
    subscriptions: string[],
    attemptId: number,
  ) {
    if (attemptId !== this.connectAttempt) {
      return;
    }

    const url = candidates[index];
    if (!url) {
      this.emitStatus("error", "No valid MQTT WebSocket endpoint");
      return;
    }

    this.emitStatus("connecting", index === 0 ? url : `${url} [fallback]`);

    const options: IClientOptions = {
      clientId: createClientId(),
      protocolVersion: 4,
      connectTimeout: 4000,
      reconnectPeriod: index === 0 ? 1000 : 0,
      reconnectOnConnackError: true,
      keepalive: 300,
      clean: true,
      resubscribe: true,
    };

    const client = mqtt.connect(url, options);
    this.client = client;
    let connected = false;

    if (this.fallbackTimer) {
      clearTimeout(this.fallbackTimer);
      this.fallbackTimer = undefined;
    }

    if (index + 1 < candidates.length) {
      this.fallbackTimer = setTimeout(() => {
        if (attemptId !== this.connectAttempt || connected || this.client !== client) {
          return;
        }
        client.end(true);
        this.tryConnect(candidates, index + 1, subscriptions, attemptId);
      }, 4500);
    }

    client.on("connect", () => {
      if (attemptId !== this.connectAttempt || this.client !== client) {
        return;
      }
      connected = true;
      if (this.fallbackTimer) {
        clearTimeout(this.fallbackTimer);
        this.fallbackTimer = undefined;
      }
      this.emitStatus("connected", url);
      if (subscriptions.length > 0) {
        client.subscribe(subscriptions, { qos: 0 });
      }
    });
    client.on("reconnect", () => {
      if (attemptId !== this.connectAttempt || this.client !== client) {
        return;
      }
      this.emitStatus("connecting", `${url} [reconnecting]`);
    });
    client.on("message", (topic, payload) => {
      if (attemptId !== this.connectAttempt || this.client !== client) {
        return;
      }
      const bytes = new Uint8Array(payload);
      let text: string | undefined;
      let json: unknown;

      try {
        text = new TextDecoder().decode(bytes);
        json = JSON.parse(text);
      } catch {
        text = undefined;
      }

      const message: VizMqttMessage = {
        topic,
        payload: bytes,
        text,
        json,
      };

      for (const listener of this.listeners) {
        listener(message);
      }
    });
    client.on("close", () => {
      if (attemptId !== this.connectAttempt || this.client !== client) {
        return;
      }
      if (client.reconnecting) {
        this.emitStatus("connecting", `${url} [reconnecting]`);
        return;
      }
      this.emitStatus("disconnected", url);
    });
    client.on("error", (error) => {
      if (attemptId !== this.connectAttempt || this.client !== client) {
        return;
      }
      this.emitStatus("error", `${url} [${error.message}]`);
    });
  }
}
