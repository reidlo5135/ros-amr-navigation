# ROS2 Based AMR Navigation Stack Mermaid Diagrams

문서 내 다이어그램 삽입 위치별 Mermaid 코드 목록임. 각 항목을 렌더링해 해당 목차 위치에 삽입하면 됨.

## 1.3 개발 로드맵

```mermaid
timeline
    title ROS2 Based AMR Navigation Stack Roadmap
    0.1.x : A* global planning MVP
          : AMCL-lite localization
          : PID motion control
    0.2.x : Costmap-based replanning
          : SLAM-lite mapping workflow
          : lifecycle bringup
    0.3.x : Dynamic obstacle split
          : centralized costmap ownership
          : total bringup stabilization
    0.4.x : Early visualization scaffold
    0.5.x : Operator architecture transition
          : robot/operator role split
          : visualization contract cleanup
    0.6.x : Direct operator tooling
          : interactive control panel
    0.7.x : Operator session recovery
          : TF/rendering stabilization
    0.8.x : Footprint-aware inflation
          : planning safety refinement
    0.9.x : BT navigator skeleton
          : escape/detour recovery flow
    0.10.x : Recovery server responsibilities
           : local costmap-driven dynamic handling
           : package consolidation
    0.11.x : Exact planning/control quality uplift
           : docs and package normalization
    0.12.x : Exact footprint navigation baseline
           : stable navigation runtime
    0.13.x : ARL / GL experimental track
           : deprecated after shared-path regression risk
    0.14.x : Mapping-line reboot
           : external SLAM temp-map consumer flow
    0.15.x : Navigation runtime hardening
           : recovery observability and regression baselines
    0.16.x : Local escape-first recovery refinement
           : corridor / doorway blocked-state tuning
           : final-approach stability cleanup
    0.17.x : Planner/controller quality uplift
           : smoother/path-handler style path quality
           : blocked semantics and path-rejoin quality
    0.18.x : Safety + task-layer navigation operations
           : near-field safety lane
           : route/waypoint command-state contract maturity
    0.19.x : Docking and deployment readiness
           : dock/undock lifecycle
           : startup/shutdown/reconnect diagnostics
    1.0.0 : Single-robot indoor AMR operational closure
```

## 2.4 시스템 컨텍스트 다이어그램

```mermaid
flowchart LR
    Operator["운영자"] --> UI["amr_visualization\nROS2 + Qt6 Operator App"]
    UI -->|ROS2 Action / Service / Topic| Runtime["AMR Navigation Runtime"]
    Runtime --> RobotIF["Robot Base Interface\n/tf / scan / odom / cmd_vel"]
    RobotIF --> Robot["Robot Hardware"]
    Robot --> Sensor["LiDAR / Odometry / TF"]
    Sensor --> Runtime

    subgraph Runtime["AMR Navigation Runtime"]
      Map["Map Server"]
      Loc["Localization"]
      Costmap["Costmap Server"]
      GPlan["Global Planner"]
      Ctrl["Controller Server\nLocal Planner + Motion Controller"]
      BT["BT Navigator"]
      Recovery["Recovery Server"]
      Obs["Runtime Observation"]
    end
```

## 3.1 아키텍처 구성도

```mermaid
flowchart TB
    UI["amr_visualization\nOperator UI"] --> BT["amr_bt_navigator"]
    UI --> Obs["amr_runtime_observation"]
    UI --> Map["amr_map_server"]

    subgraph NavRuntime["ROS2 Based AMR Navigation Runtime"]
        Map --> Loc["amr_localization"]
        Loc --> Costmap["amr_costmap_server"]
        Costmap --> GPlan["amr_global_planner"]
        GPlan --> BT
        BT --> Ctrl["amr_controller_server"]
        Ctrl --> Local["/amr/local_planner"]
        Ctrl --> Motion["/amr/motion_controller"]
        Local --> Motion
        Motion --> Base["/cmd_vel"]
        Local --> BT
        Motion --> BT
        Recovery["amr_recovery_server"] --> BT
        BT --> Recovery
        BT --> Obs
        Local --> Obs
        Motion --> Obs
    end

    Sensors["/scan / odom / tf"] --> Loc
    Sensors --> Costmap
    Base --> Robot["Robot Base"]
```

## 4.1 패키지 구성도

```mermaid
flowchart LR
    Bringup["amr_bringup\nlaunch + params"] --> Lifecycle["amr_lifecycle_manager"]
    Bringup --> NavPkg["amr_navigation\nmetapackage"]
    Msgs["amr_msgs\ninterfaces"] --> BT
    Msgs --> Planner
    Msgs --> Controller
    Msgs --> Recovery
    Msgs --> UI

    subgraph Core["Core Navigation Packages"]
        Map["amr_map_server"]
        Loc["amr_localization"]
        Costmap["amr_costmap_server"]
        Planner["amr_global_planner"]
        Controller["amr_controller_server"]
        Recovery["amr_recovery_server"]
        BT["amr_bt_navigator"]
        Obs["amr_runtime_observation"]
    end

    subgraph Operator["Operator / Test Packages"]
        UI["amr_visualization"]
        RViz["amr_rviz"]
    end

    Geometry["amr_geometry\nfootprint/collision helpers"] --> Planner
    Geometry --> Controller
```

## 4.2 실행 환경 구성도

```mermaid
flowchart LR
    subgraph RemotePC["Remote PC\nUbuntu execution/test environment"]
        UI["amr_visualization"]
        Stack["amr_navigation runtime\nmap/localization/costmap/planner/controller/BT/recovery/observation"]
    end

    subgraph Robot["Robot\nmobile base environment"]
        TB3["robot bringup\nbase driver / sensors / TF"]
        Lidar["/scan"]
        Odom["/odom"]
        Cmd["/cmd_vel"]
    end

    UI -->|ROS2| Stack
    Lidar -->|ROS2 DDS| Stack
    Odom -->|ROS2 DDS| Stack
    Stack -->|ROS2 DDS| Cmd
    TB3 --> Lidar
    TB3 --> Odom
    Cmd --> TB3

    Common["Common ROS2 Runtime\nshared RMW + same domain + same time/TF convention"]
    Common -.-> RemotePC
    Common -.-> Robot
```

## 5.1 Localization 흐름도

```mermaid
flowchart LR
    Odom["/odom"] --> Loc["amr_localization"]
    Scan["/scan"] --> Loc
    Map["/amr/map/data"] --> Loc
    Init["/amr/localization/initial_pose"] --> Loc
    Loc --> Pose["/amr/localization/pose"]
    Loc --> LOdom["/amr/localization/odometry"]
    Loc --> TF["TF map -> odom"]
```

## 5.2 Map lifecycle 흐름도

```mermaid
flowchart TD
    YAML["map yaml/pgm"] --> MapServer["amr_map_server"]
    Temp["temporary SLAM map"] --> MapServer
    Corr["corrected odometry"] --> MapServer
    MapServer --> Official["/amr/map/data"]
    MapServer --> Get["get_map service"]
    MapServer --> Freeze["freeze temporary map"]
    MapServer --> Evaluate["evaluate temporary map"]
    MapServer --> Save["save temporary map"]
    Evaluate --> Quality["known/free/occupied/inflated-free ratio check"]
    Save --> Files["saved map yaml/pgm"]
```

## 5.3 Costmap 흐름도

```mermaid
flowchart LR
    Map["/amr/map/data"] --> Costmap["amr_costmap_server"]
    Pose["/amr/localization/pose"] --> Costmap
    Scan["/scan"] --> Costmap
    Costmap --> Global["/amr/costmap/global\nstatic inflated map"]
    Costmap --> Local["/amr/costmap/local\nscan-based local window"]
    Clear["clear_costmap service"] --> Costmap
```

## 5.4 Global planning 흐름도

```mermaid
flowchart TD
    Request["PlanSegment / PlanRoute request"] --> Planner["amr_global_planner"]
    Costmap["/amr/costmap/global"] --> Planner
    Planner --> Bounds["world -> grid 변환"]
    Bounds --> Free["nearest free cell search"]
    Free --> AStar["A* search\nconnectivity / turn penalty / no corner cutting"]
    AStar --> Footprint["exact footprint collision check"]
    Footprint --> Path["nav_msgs/Path"]
    Path --> Publish["/amr/planner/global"]
```

## 5.5 Local planning and path refinement 흐름도

```mermaid
flowchart TD
    Command["/amr/motion/command"] --> Local["/amr/local_planner"]
    Pose["/amr/localization/pose"] --> Local
    LCostmap["/amr/costmap/local"] --> Local
    Local --> Slice["global path slice / lookahead"]
    Slice --> Refine["path_refiner\nprune / interpolate / heading / corner smoothing"]
    Refine --> Collision["collision validation"]
    Collision -->|safe| Plan["/amr/planner/local"]
    Collision -->|unsafe| Fallback["fallback to unsmoothed path"]
    Local --> Status["/amr/planner/local_status"]
    Status --> Decision["OK / goal proximity / global replan / hard blocked"]
```

## 5.6 Motion control and goal checking 흐름도

```mermaid
flowchart TD
    Command["/amr/motion/command"] --> Motion["/amr/motion_controller"]
    LocalPlan["/amr/planner/local"] --> Motion
    Pose["/amr/localization/pose"] --> Motion
    Scan["/scan"] --> Safety["safety_gate"]
    Safety --> Motion
    Motion --> Track["tracking target selection"]
    Track --> GoalCheck["goal_checker\nXY / yaw / hold-time"]
    GoalCheck --> PID["velocity_controller\nP/PI/PID + accel limits"]
    PID --> CmdVel["/cmd_vel"]
    Motion --> Status["/amr/motion/status"]
```

## 5.7 BT navigation and recovery orchestration 흐름도

```mermaid
flowchart TD
    Goal["NavigateToPose / NavigateToPoses"] --> BT["amr_bt_navigator"]
    BT --> Ready["ready + current pose check"]
    Ready --> Plan["request global plan"]
    Plan --> Dispatch["publish MotionCommand"]
    Dispatch --> Monitor["monitor motion/local status"]
    Monitor -->|goal reached| Success["action success"]
    Monitor -->|cancel| Cancel["action cancel"]
    Monitor -->|blocked/stalled| RecoveryDecision["recovery decision"]
    RecoveryDecision --> Escape["one local escape if planner-owned hard blocked"]
    RecoveryDecision --> RecSrv["wait / backup / spin"]
    RecoveryDecision --> Replan["fresh global replan"]
    Escape --> Dispatch
    RecSrv --> Dispatch
    Replan --> Dispatch
```

## 5.8 Runtime observation 흐름도

```mermaid
flowchart LR
    MotionCommand["/amr/motion/command"] --> Obs["amr_runtime_observation"]
    MotionStatus["/amr/motion/status"] --> Obs
    LocalStatus["/amr/planner/local_status"] --> Obs
    ActionFeedback["NavigateToPoses feedback/status"] --> Obs
    Obs --> Summary["/amr/observation/runtime/summary"]
    Obs --> Events["/amr/observation/runtime/events"]
    Summary --> UI["amr_visualization"]
    Events --> UI
```

## 5.9 Operator visualization 흐름도

```mermaid
flowchart TD
    UI["amr_visualization\nQt6 Operator App"] --> Initial["initial pose command"]
    UI --> Goal["single/multi goal command"]
    UI --> Monitor["runtime monitoring"]
    Map["/amr/map/data"] --> UI
    Pose["/amr/localization/pose"] --> UI
    GPath["/amr/planner/global"] --> UI
    LPath["/amr/planner/local"] --> UI
    Summary["runtime summary/events"] --> UI
    UI --> Action["/amr/navigator actions"]
    UI --> InitTopic["/amr/localization/initial_pose"]
```

## 6.1 Full runtime bringup 시퀀스

```mermaid
sequenceDiagram
    box Robot(turtlebot3)
      participant Launch as bringup launch
      participant Robot as robot bringup
    end

    box RemotePC(BoxPC)
      participant LM1 as localization_manager
      participant LM2 as navigation_manager
      participant Core as lifecycle nodes
    end

    Launch->>Robot: base/sensor/TF bringup
    Launch->>LM1: map/localization/costmap/global planner bringup
    LM1->>Core: configure
    LM1->>Core: activate
    LM1->>Core: initial pose publish when enabled
    Launch->>LM2: controller/recovery/navigator bringup
    LM2->>Core: configure + activate
```

## 6.2 Single-goal navigation 시퀀스

```mermaid
sequenceDiagram
    participant UI as Operator UI
    participant BT as BT Navigator
    participant GP as Global Planner
    participant LP as Local Planner
    participant MC as Motion Controller
    UI->>BT: NavigateToPose(goal)
    BT->>GP: PlanSegment(current, goal)
    GP-->>BT: global path
    BT->>LP: MotionCommand(plan, goal)
    LP-->>MC: local plan
    MC-->>BT: MotionStatus
    BT-->>UI: feedback/result
```

## 6.3 Multi-goal route navigation 시퀀스

```mermaid
sequenceDiagram
    participant UI as Operator UI
    participant BT as BT Navigator
    participant GP as Global Planner
    participant CTRL as Controller Server
    UI->>BT: NavigateToPoses(goal_poses[])
    loop each waypoint
      BT->>GP: PlanSegment(current, waypoint)
      GP-->>BT: segment path
      BT->>CTRL: MotionCommand(segment)
      CTRL-->>BT: status/goal reached
      BT-->>UI: current_goal_index feedback
    end
    BT-->>UI: completed_goals result
```

## 6.4 Dynamic obstacle and blocked-state handling 흐름도

```mermaid
flowchart TD
    LocalCostmap["local costmap update"] --> LocalPlanner["local planner check"]
    LocalPlanner --> Decision{decision}
    Decision -->|OK| Continue["continue path tracking"]
    Decision -->|GOAL_PROXIMITY_BLOCKED| Wait["wait + re-dispatch"]
    Decision -->|GLOBAL_REPLAN_REQUIRED| Replan["fresh global replan"]
    Decision -->|HARD_BLOCKED| Confirm["blocked-distance-aware confirmation"]
    Confirm --> Escape["one local escape"]
    Escape --> Fallback["backup -> spin -> replan if needed"]
```

## 6.5 Recovery flow 시퀀스

```mermaid
sequenceDiagram
    participant BT as BT Navigator
    participant LP as Local Planner
    participant RS as Recovery Server
    participant MC as Motion Controller
    BT->>LP: PlanLocalEscape(current, source_plan)
    alt local escape ready
      LP-->>BT: escape plan
      BT->>MC: MotionCommand(MODE_NAVIGATE)
    else local escape unavailable or not applicable
      BT->>RS: PlanRecovery(wait/backup/spin)
      RS-->>BT: recovery MotionCommand
      BT->>MC: MotionCommand(recovery)
    end
    MC-->>BT: status
    BT->>BT: fresh global replan if bounded recovery exhausted
```

## 6.6 Final approach and goal completion 흐름도

```mermaid
flowchart TD
    NearGoal["near goal"] --> XY["XY tolerance check"]
    XY -->|not reached| Track["continue tracking"]
    XY -->|reached| AlignPolicy{align heading?}
    AlignPolicy -->|false| Success["goal reached"]
    AlignPolicy -->|true| Heading["final heading settle"]
    Heading --> Deadband["tracking heading deadband + max angular cap"]
    Deadband --> Success
```

## 6.7 Map freeze / evaluate / save flow

```mermaid
sequenceDiagram
    participant UI as Operator UI
    participant Map as Map Server
    participant SLAM as Temporary map source
    SLAM-->>Map: temporary refined map
    UI->>Map: evaluate_temporary_map
    Map-->>UI: quality metrics result
    UI->>Map: freeze_temporary_map
    Map-->>UI: official map promoted
    UI->>Map: save_temporary_map
    Map-->>UI: saved yaml/pgm result
```

## 6.8 Operator monitoring flow

```mermaid
flowchart LR
    Map["map"] --> UI["amr_visualization"]
    Pose["pose"] --> UI
    Paths["global/local paths"] --> UI
    Status["motion + local planner status"] --> Obs["runtime observation"]
    Obs --> Summary["summary/events"]
    Summary --> UI
    UI --> Operator["operator decision"]
```

## 7.2 핵심 인터페이스 맵

```mermaid
flowchart TB
    UI["amr_visualization"] -->|Action| BT["/amr/navigator"]
    UI -->|Topic| Init["/amr/localization/initial_pose"]
    BT -->|Service| GP["/amr/global_planner/plan_segment"]
    BT -->|Service| LE["/amr/local_planner/plan_local_escape"]
    BT -->|Service| RS["/amr/recovery_server/plan_recovery"]
    BT -->|Topic| Cmd["/amr/motion/command"]
    LP["/amr/local_planner"] -->|Topic| LPlan["/amr/planner/local"]
    LP -->|Topic| LStatus["/amr/planner/local_status"]
    MC["/amr/motion_controller"] -->|Topic| MStatus["/amr/motion/status"]
    Obs["/amr/runtime_observation"] -->|Topic| Summary["summary/events"]
```

