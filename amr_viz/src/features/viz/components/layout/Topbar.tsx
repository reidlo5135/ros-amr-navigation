type TopbarProps = {
  connectionLabel: string;
  poseLabel: string;
  batteryPercentage: number | null;
};

export function Topbar({ connectionLabel, poseLabel, batteryPercentage }: TopbarProps) {
  return (
    <header className="topbar">
      <div className="topbar-title">AMR Viz</div>
      <div className="topbar-status">
        <span className="topbar-chip">Fixed Frame: map</span>
        <span className="topbar-chip">{connectionLabel}</span>
        <span className="topbar-chip">Pose {poseLabel}</span>
        <span className="topbar-chip battery-chip" aria-label="Battery status">
          <span className="battery-shell">
            <span
              className={`battery-fill${batteryPercentage !== null && batteryPercentage <= 20 ? " low" : ""}`}
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
