import type { BridgeCommand, BridgeEnvelope } from "./protocol";

type Listener = (message: BridgeEnvelope) => void;
type StatusListener = (
  status: "connecting" | "connected" | "disconnected" | "error",
  detail?: string,
) => void;

export class VizSocketClient {
  private websocket?: WebSocket;
  private listeners = new Set<Listener>();
  private statusListeners = new Set<StatusListener>();

  connect(url: string) {
    this.disconnect();
    this.emitStatus("connecting", url);
    this.websocket = new WebSocket(url);
    this.websocket.addEventListener("open", () => {
      this.emitStatus("connected", url);
    });
    this.websocket.addEventListener("message", (event) => {
      const parsed = JSON.parse(String(event.data)) as BridgeEnvelope;
      for (const listener of this.listeners) {
        listener(parsed);
      }
    });
    this.websocket.addEventListener("close", (event) => {
      const reason = event.reason ? ` (${event.reason})` : "";
      this.emitStatus("disconnected", `${url} [code=${event.code}]${reason}`);
    });
    this.websocket.addEventListener("error", () => {
      this.emitStatus("error", `${url} [handshake failed or server unavailable]`);
    });
  }

  disconnect() {
    if (this.websocket) {
      this.websocket.close();
      this.websocket = undefined;
    }
    this.emitStatus("disconnected");
  }

  send(command: BridgeCommand) {
    if (!this.websocket || this.websocket.readyState !== WebSocket.OPEN) {
      return false;
    }
    this.websocket.send(JSON.stringify(command));
    return true;
  }

  onMessage(listener: Listener) {
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

  get readyState() {
    return this.websocket?.readyState ?? WebSocket.CLOSED;
  }
}
