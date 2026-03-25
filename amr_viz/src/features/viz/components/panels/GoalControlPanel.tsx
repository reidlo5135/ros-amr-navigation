import type { InteractionMode } from "../../types";

type GoalControlPanelProps = {
  goalX: string;
  goalY: string;
  goalYaw: string;
  interactionMode: InteractionMode;
  onGoalXChange: (value: string) => void;
  onGoalYChange: (value: string) => void;
  onGoalYawChange: (value: string) => void;
  onToggleGoalMode: () => void;
  onToggleInitialPoseMode: () => void;
  onCancelGoal: () => void;
};

export function GoalControlPanel({
  goalX,
  goalY,
  goalYaw,
  interactionMode,
  onGoalXChange,
  onGoalYChange,
  onGoalYawChange,
  onToggleGoalMode,
  onToggleInitialPoseMode,
  onCancelGoal,
}: GoalControlPanelProps) {
  return (
    <section className="panel-card">
      <div className="panel-section-title">Tools</div>
      <div className="field-grid field-grid-triple">
        <label className="field-label">
          <span>X</span>
          <input value={goalX} onChange={(event) => onGoalXChange(event.target.value)} />
        </label>
        <label className="field-label">
          <span>Y</span>
          <input value={goalY} onChange={(event) => onGoalYChange(event.target.value)} />
        </label>
        <label className="field-label">
          <span>Yaw</span>
          <input value={goalYaw} onChange={(event) => onGoalYawChange(event.target.value)} />
        </label>
      </div>
      <div className="button-row">
        <button
          className={interactionMode === "goal" ? "active-mode" : undefined}
          onClick={onToggleGoalMode}
        >
          Send
        </button>
        <button className="danger" onClick={onCancelGoal}>
          Cancel
        </button>
      </div>
      <div className="button-row single-row">
        <button
          className={`secondary${interactionMode === "initial_pose" ? " active-mode" : ""}`}
          onClick={onToggleInitialPoseMode}
        >
          Set Initial Pose
        </button>
      </div>
      <div className="mode-hint">
        Select `Send` or `Set Initial Pose`, then left-drag on map to apply. Map interaction also updates
        X / Y / Yaw.
      </div>
    </section>
  );
}
