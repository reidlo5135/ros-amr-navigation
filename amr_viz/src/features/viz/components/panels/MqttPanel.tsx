import { useEffect, useState } from "react";

type MqttPanelProps = {
  mqttUrl: string;
  robotId: string;
  onMqttUrlChange: (value: string) => void;
  onRobotIdChange: (value: string) => void;
  onConnect: () => void;
  onDisconnect: () => void;
};

export function MqttPanel({
  mqttUrl,
  robotId,
  onMqttUrlChange,
  onRobotIdChange,
  onConnect,
  onDisconnect,
}: MqttPanelProps) {
  const [robotIdDraft, setRobotIdDraft] = useState(robotId);

  useEffect(() => {
    setRobotIdDraft(robotId);
  }, [robotId]);

  const applyRobotIdDraft = () => {
    onRobotIdChange(robotIdDraft);
  };

  return (
    <section className="panel-card">
      <div className="panel-section-title">Global Options</div>
      <label className="field-label">
        <span>Broker WS</span>
        <input value={mqttUrl} onChange={(event) => onMqttUrlChange(event.target.value)} />
      </label>
      <label className="field-label">
        <span>Robot ID</span>
        <input
          value={robotIdDraft}
          onChange={(event) => setRobotIdDraft(event.target.value)}
          onBlur={applyRobotIdDraft}
          onKeyDown={(event) => {
            if (event.key === "Enter") {
              applyRobotIdDraft();
            }
          }}
        />
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
