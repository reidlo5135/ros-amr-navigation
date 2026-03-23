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

export class VizMqttClient {
  private client?: MqttClient;
  private listeners = new Set<MessageListener>();
  private statusListeners = new Set<StatusListener>();

  connect(url: string, subscriptions: string[]) {
    this.disconnect();
    this.emitStatus("connecting", url);

    const options: IClientOptions = {
      connectTimeout: 3000,
      reconnectPeriod: 0,
      keepalive: 20,
      clean: true,
    };

    this.client = mqtt.connect(url, options);
    this.client.on("connect", () => {
      this.emitStatus("connected", url);
      if (subscriptions.length > 0) {
        this.client?.subscribe(subscriptions, { qos: 0 });
      }
    });
    this.client.on("message", (topic, payload) => {
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
    this.client.on("close", () => {
      this.emitStatus("disconnected", url);
    });
    this.client.on("error", (error) => {
      this.emitStatus("error", `${url} [${error.message}]`);
    });
  }

  disconnect() {
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
}
