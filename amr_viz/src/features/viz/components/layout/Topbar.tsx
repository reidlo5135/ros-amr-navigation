type TopbarProps = {
  connectionLabel: string;
  poseLabel: string;
  viewMode: "nav" | "mapping";
  onViewModeChange: (mode: "nav" | "mapping") => void;
  signalBars: number;
  signalRttMs: number | null;
  batteryPercentage: number | null;
};

export function Topbar({
  connectionLabel,
  poseLabel,
  viewMode,
  onViewModeChange,
  signalBars,
  signalRttMs,
  batteryPercentage,
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
        <span className="topbar-chip">Pose {poseLabel}</span>
        <span className="topbar-chip mode-chip">
          <button
            type="button"
            className={`mode-button${viewMode === "nav" ? " active" : ""}`}
            onClick={() => onViewModeChange("nav")}
          >
            Nav
          </button>
          <button
            type="button"
            className={`mode-button${viewMode === "mapping" ? " active" : ""}`}
            onClick={() => onViewModeChange("mapping")}
          >
            Mapping
          </button>
        </span>
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
