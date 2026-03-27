type LocalizationDebugPanelProps = {
  hasRobotPose: boolean;
  hasOdom: boolean;
  hasFootprint: boolean;
  hasMapToOdom: boolean;
};

function renderStatusLabel(value: boolean) {
  return value ? "OK" : "Missing";
}

export function LocalizationDebugPanel({
  hasRobotPose,
  hasOdom,
  hasFootprint,
  hasMapToOdom,
}: LocalizationDebugPanelProps) {
  return (
    <section className="panel-card">
      <div className="panel-section-title">Localization Debug</div>
      <div className="metric-list">
        <div className="metric-row">
          <span>LPose</span>
          <strong className={hasRobotPose ? "metric-status-ok" : "metric-status-missing"}>
            {renderStatusLabel(hasRobotPose)}
          </strong>
        </div>
        <div className="metric-row">
          <span>Odom</span>
          <strong className={hasOdom ? "metric-status-ok" : "metric-status-missing"}>
            {renderStatusLabel(hasOdom)}
          </strong>
        </div>
        <div className="metric-row">
          <span>Footprint</span>
          <strong className={hasFootprint ? "metric-status-ok" : "metric-status-missing"}>
            {renderStatusLabel(hasFootprint)}
          </strong>
        </div>
        <div className="metric-row">
          <span>map→odom</span>
          <strong className={hasMapToOdom ? "metric-status-ok" : "metric-status-missing"}>
            {renderStatusLabel(hasMapToOdom)}
          </strong>
        </div>
      </div>
    </section>
  );
}
