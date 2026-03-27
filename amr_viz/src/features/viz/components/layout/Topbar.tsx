import type { VisualizationMode } from "../../types";

type TopbarProps = {
  connectionLabel: string;
  poseLabel: string;
  signalBars: number;
  signalRttMs: number | null;
  batteryPercentage: number | null;
  visualizationMode: VisualizationMode;
  onVisualizationModeChange: (mode: VisualizationMode) => void;
};

export function Topbar({
  connectionLabel,
  poseLabel,
  signalBars,
  signalRttMs,
  batteryPercentage,
  visualizationMode,
  onVisualizationModeChange,
}: TopbarProps) {
  const batteryLevelClass =
    batteryPercentage === null ? "" :
    batteryPercentage > 80 ? " high" :
    batteryPercentage >= 40 ? " medium" :
    " low";

  return (
    <header className="topbar">
      <div className="topbar-title">AMR Viz</div>
      <div className="topbar-status">
        <span className="topbar-chip">Fixed Frame: map</span>
        <span className="topbar-chip">{connectionLabel}</span>
        <span className="topbar-chip topbar-mode-toggle">
          <button
            type="button"
            className={visualizationMode === "arl" ? "active-mode" : ""}
            onClick={() => onVisualizationModeChange("arl")}
          >
            ARL
          </button>
          <button
            type="button"
            className={visualizationMode === "nav" ? "active-mode secondary" : "secondary"}
            onClick={() => onVisualizationModeChange("nav")}
          >
            Nav
          </button>
        </span>
        <span className="topbar-chip">Pose {poseLabel}</span>
        <span className="topbar-chip signal-chip" aria-label="MQTT signal status">
          <span className="signal-bars" aria-hidden="true">
            {[0, 1, 2, 3].map((index) => (
              <span key={index} className={`signal-bar${signalBars > index ? " active" : ""}`} />
            ))}
          </span>
          <span className="signal-label">{signalRttMs !== null ? `${Math.round(signalRttMs)} ms` : "--"}</span>
        </span>
        <span className="topbar-chip battery-chip" aria-label="Battery status">
          <span className="battery-shell">
            <span
              className={`battery-fill${batteryLevelClass}`}
              style={{ width: `${batteryPercentage ?? 0}%` }}
            />
            <span className="battery-tip" />
          </span>
          <span className="battery-label">
            {batteryPercentage !== null ? `${Math.round(batteryPercentage)}%` : "--%"}
          </span>
        </span>
      </div>
    </header>
  );
}
