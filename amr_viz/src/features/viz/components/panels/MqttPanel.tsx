type MqttPanelProps = {
  mqttUrl: string;
  onMqttUrlChange: (value: string) => void;
  onConnect: () => void;
  onDisconnect: () => void;
};

export function MqttPanel({
  mqttUrl,
  onMqttUrlChange,
  onConnect,
  onDisconnect,
}: MqttPanelProps) {
  return (
    <section className="panel-card">
      <div className="panel-section-title">Global Options</div>
      <label className="field-label">
        <span>Broker WS</span>
        <input value={mqttUrl} onChange={(event) => onMqttUrlChange(event.target.value)} />
      </label>
      <div className="button-row">
        <button onClick={onConnect}>Connect</button>
        <button className="secondary" onClick={onDisconnect}>
          Disconnect
        </button>
      </div>
    </section>
  );
}
