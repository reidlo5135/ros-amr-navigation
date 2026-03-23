import type { BridgeState } from "../lib/protocol";

type StatusPanelProps = {
  state: BridgeState;
  connectionLabel: string;
};

export function StatusPanel({ state, connectionLabel }: StatusPanelProps) {
  const robotPose = state.robot_pose;
  const motionStatus = state.motion_status;
  const obstacleReport = state.obstacle_report;

  return (
    <section className="status-panel">
      <div className="panel-card">
        <div className="panel-label">Bridge</div>
        <div className="panel-value">{connectionLabel}</div>
      </div>
      <div className="panel-card">
        <div className="panel-label">Robot</div>
        <div className="panel-value">
          {robotPose
            ? `${robotPose.position.x.toFixed(2)}, ${robotPose.position.y.toFixed(2)}`
            : "No pose"}
        </div>
      </div>
      <div className="panel-card">
        <div className="panel-label">Motion</div>
        <div className="panel-value">
          {motionStatus?.active ? "Navigating" : "Idle"}
        </div>
        <div className="panel-note">
          Remaining {motionStatus?.remaining_distance?.toFixed(2) ?? "--"} m
        </div>
      </div>
      <div className="panel-card">
        <div className="panel-label">Obstacle</div>
        <div className="panel-value">
          {obstacleReport?.active
            ? `${obstacleReport.is_dynamic ? "Dynamic" : "Static"} ${obstacleReport.distance.toFixed(2)} m`
            : "Clear"}
        </div>
        <div className="panel-note">
          {obstacleReport?.blocks_path ? "Path blocked" : "Path available"}
        </div>
      </div>
    </section>
  );
}
