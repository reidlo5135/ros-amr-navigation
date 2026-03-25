import type { MotionStatusMessage } from "../../../../lib/protocol";
import type { GoalLifecycleState } from "../../types";

type NavigationStatusPanelProps = {
  motionStatus?: MotionStatusMessage;
  goalLifecycle: GoalLifecycleState;
};

export function NavigationStatusPanel({
  motionStatus,
  goalLifecycle,
}: NavigationStatusPanelProps) {
  return (
    <section className="panel-card">
      <div className="panel-section-title">Navigation Status</div>
      <div className="metric-list">
        <div className="metric-row">
          <span>Motion</span>
          <strong>{motionStatus?.active ? "Navigating" : "Idle"}</strong>
        </div>
        <div className="metric-row">
          <span>Remaining</span>
          <strong>{motionStatus?.remaining_distance?.toFixed(2) ?? "--"} m</strong>
        </div>
        <div className="metric-row">
          <span>Heading</span>
          <strong>{motionStatus?.heading_error?.toFixed(2) ?? "--"} rad</strong>
        </div>
        <div className="metric-row">
          <span>Goal</span>
          <strong>{goalLifecycle}</strong>
        </div>
        <div className="metric-row">
          <span>Blocked Source</span>
          <strong>
            {motionStatus?.costmap_blocked ? "Costmap" :
              motionStatus?.safety_gate_blocked ? "Safety Gate" :
              motionStatus?.blocked ? "Blocked" : "Clear"}
          </strong>
        </div>
      </div>
    </section>
  );
}
