import { SceneViewport } from "../components/scene/SceneViewport";
import { Topbar } from "../components/layout/Topbar";
import { EventsPanel } from "../components/panels/EventsPanel";
import { GoalControlPanel } from "../components/panels/GoalControlPanel";
import { JoystickPanel } from "../components/panels/JoystickPanel";
import { LayersPanel } from "../components/panels/LayersPanel";
import { MapSavePanel } from "../components/panels/MapSavePanel";
import { MqttPanel } from "../components/panels/MqttPanel";
import { NavigationStatusPanel } from "../components/panels/NavigationStatusPanel";
import { useVizDashboard } from "../hooks/useVizDashboard";

import "../styles/dashboard.css";
import "../styles/panels.css";
import "../styles/topbar.css";

export function VizDashboardPage() {
  const dashboard = useVizDashboard();
  const activePose =
    dashboard.viewMode === "mapping"
      ? dashboard.bridgeState.mapping_pose ?? dashboard.bridgeState.robot_pose
      : dashboard.bridgeState.robot_pose ?? dashboard.bridgeState.mapping_pose;
  const poseLabel = activePose
    ? `${activePose.position.x.toFixed(2)}, ${activePose.position.y.toFixed(2)}`
    : "--";

  return (
    <main className="app-shell">
      <Topbar
        connectionLabel={dashboard.connectionLabel}
        poseLabel={poseLabel}
        viewMode={dashboard.viewMode}
        onViewModeChange={dashboard.setViewMode}
        signalBars={dashboard.signalBars}
        signalRttMs={dashboard.signalRttMs}
        batteryPercentage={dashboard.batteryPercentage}
      />

      <section className="workspace">
        <aside className="sidebar sidebar-left">
          <MqttPanel
            mqttUrl={dashboard.mqttUrl}
            robotId={dashboard.robotId}
            onMqttUrlChange={dashboard.setMqttUrl}
            onRobotIdChange={dashboard.setRobotId}
            onConnect={dashboard.connect}
            onDisconnect={dashboard.disconnect}
          />
          <GoalControlPanel
            goalX={dashboard.goalX}
            goalY={dashboard.goalY}
            goalYaw={dashboard.goalYaw}
            interactionMode={dashboard.interactionMode}
            onGoalXChange={dashboard.setGoalX}
            onGoalYChange={dashboard.setGoalY}
            onGoalYawChange={dashboard.setGoalYaw}
            onToggleGoalMode={dashboard.toggleGoalMode}
            onToggleInitialPoseMode={dashboard.toggleInitialPoseMode}
            onCancelGoal={dashboard.cancelGoal}
          />
          <LayersPanel
            viewMode={dashboard.viewMode}
            layerVisibility={dashboard.layerVisibility}
            onToggleLayer={dashboard.toggleLayer}
          />
          {dashboard.viewMode === "mapping" && (
            <MapSavePanel
              basename={dashboard.mapSaveBasename}
              onBasenameChange={dashboard.setMapSaveBasename}
              onSave={dashboard.saveMappingMap}
            />
          )}
        </aside>

        <section className="scene-panel">
          <div className="scene-toolbar">
            <span className="toolbar-label">Scene</span>
            <span className="toolbar-value">Background: 228; 228; 228</span>
            <span className="toolbar-value">
              Map {dashboard.bridgeState.map ? `${dashboard.bridgeState.map.info.width}x${dashboard.bridgeState.map.info.height}` : "--"}
            </span>
            <span className="toolbar-value">
              Raw {dashboard.bridgeState.temp_map_raw ? `${dashboard.bridgeState.temp_map_raw.info.width}x${dashboard.bridgeState.temp_map_raw.info.height}` : "--"}
            </span>
            <span className="toolbar-value">
              Refined {dashboard.bridgeState.temp_map_refined ? `${dashboard.bridgeState.temp_map_refined.info.width}x${dashboard.bridgeState.temp_map_refined.info.height}` : "--"}
            </span>
            <span className="toolbar-value">
              Global {dashboard.bridgeState.global_path?.poses.length ?? 0} pts
            </span>
            <span className="toolbar-value">
              Local {dashboard.bridgeState.local_path?.poses.length ?? 0} pts
            </span>
            <span className="toolbar-value">
              Scan {dashboard.bridgeState.scan?.ranges.length ?? 0} rays
            </span>
            <span className="toolbar-value">
              Graph {dashboard.bridgeState.slam_graph?.node_count ?? 0}N/{dashboard.bridgeState.slam_graph?.edge_count ?? 0}E
            </span>
            <span className="toolbar-value">
              TF {(dashboard.bridgeState.tf?.transforms.length ?? 0) + (dashboard.bridgeState.tf_static?.transforms.length ?? 0)} frames
            </span>
          </div>
          <SceneViewport
            state={dashboard.bridgeState}
            viewMode={dashboard.viewMode}
            layerVisibility={dashboard.layerVisibility}
            goalMarker={dashboard.goalMarker}
            goalLifecycle={dashboard.resolvedGoalLifecycle}
            interactionMode={dashboard.interactionMode}
            onPoseSelection={dashboard.handleScenePoseSelection}
            onPosePlacement={dashboard.handleScenePosePlacement}
          />
        </section>

        <aside className="sidebar sidebar-right">
          <NavigationStatusPanel
            motionStatus={dashboard.bridgeState.motion_status}
            goalLifecycle={dashboard.resolvedGoalLifecycle}
          />
          <EventsPanel events={dashboard.events} />
          <JoystickPanel
            linearX={dashboard.teleopLinearX}
            angularZ={dashboard.teleopAngularZ}
            onCommandChange={dashboard.updateTeleopCommand}
            onCommandStop={dashboard.stopTeleopCommand}
          />
        </aside>
      </section>
    </main>
  );
}
