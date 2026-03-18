# ros-amr-navigation

Goal-driven ROS 2 AMR navigation stack for occupancy-grid maps, pose estimation, A* planning, local replanning, and velocity control.

## Packages

- `amr_navigation`
  - entry metapackage for building the full stack with `colcon build --packages-up-to amr_navigation`
- `amr_bringup`
  - owns launch files, shared runtime parameters, RViz config, and helper scripts
- `amr_msgs`
  - shared actions, services, and messages
- `amr_map_server`
  - owns the official navigation map and the temporary mapping map
- `amr_localization`
  - estimates the robot pose from `/odom`, `/scan`, and the static map
- `amr_global_planner`
  - computes inflated-grid A* plans for goal requests
- `amr_local_planner`
  - builds a local inflated costmap and replans a short-horizon path
- `amr_motion_controller`
  - tracks the local plan and publishes `/cmd_vel`
- `amr_bt_navigator`
  - accepts `NavigateToPose`, requests a global plan, and dispatches motion commands
- `amr_rviz_plugins`
  - bridges RViz `2D Goal Pose` input to the AMR action server

## Execution Flow

```mermaid
sequenceDiagram
    actor User
    participant RViz as amr_rviz_plugins
    participant Nav as amr_bt_navigator
    participant Global as amr_global_planner
    participant Local as amr_local_planner
    participant Motion as amr_motion_controller
    participant Base as base driver
    participant Loc as amr_localization

    User->>RViz: 2D Goal Pose / CLI goal
    RViz->>Nav: NavigateToPose(goal_pose)
    Nav->>Loc: read current pose
    Nav->>Global: PlanSegment(start, goal)
    Global-->>Nav: global path
    Nav->>Local: MotionCommand(plan, goal_pose)
    loop Control cycle
        Local->>Loc: read current pose
        Local->>Local: local costmap + local replan
        Local-->>Motion: local path
        Motion->>Loc: read current pose
        Motion->>Base: /cmd_vel
        Motion-->>Nav: MotionStatus
    end
    Nav-->>User: action result
```

## Algorithms

This section summarizes the major runtime features of the AMR stack and the algorithm flow behind each one.  
이 섹션은 현재 AMR 스택의 주요 기능과 각 기능의 알고리즘 흐름을 요약합니다.

### 1. Goal Navigation

English:
- The operator sends a goal from RViz or CLI.
- The navigator requests an inflated-grid A* global plan.
- The local planner builds a short-horizon path on the local costmap.
- The motion controller tracks that path and publishes `/cmd_vel`.

한국어:
- 사용자가 RViz 또는 CLI로 goal을 보냅니다.
- navigator가 inflation이 적용된 grid A* global plan을 요청합니다.
- local planner가 local costmap 기반 단거리 경로를 다시 계산합니다.
- motion controller가 그 경로를 추종하며 `/cmd_vel`을 발행합니다.

```mermaid
sequenceDiagram
    actor User
    participant RViz as RViz / CLI
    participant Bridge as amr_rviz_plugins
    participant Nav as amr_bt_navigator
    participant Global as amr_global_planner
    participant Local as amr_local_planner
    participant Motion as amr_motion_controller
    participant Base as base driver

    User->>RViz: Send goal
    RViz->>Bridge: PoseStamped
    Bridge->>Nav: NavigateToPose
    Nav->>Global: PlanSegment(start, goal)
    Global-->>Nav: Global path
    Nav->>Local: MotionCommand(global path, goal)
    loop control cycle
        Local-->>Motion: Local path
        Motion->>Base: /cmd_vel
        Motion-->>Nav: MotionStatus
    end
    Nav-->>User: Goal result
```

### 2. Bootstrap Mapping With Odom, IMU, and Scan Matching

English:
- Mapping mode starts from a temporary occupancy grid.
- `/odom` provides position prediction.
- `/imu` stabilizes yaw.
- Local scan matching refines the predicted pose before each scan is inserted.
- The corrected pose is also used to publish `map -> odom` for RViz alignment.

한국어:
- mapping mode는 temporary occupancy grid에서 시작합니다.
- `/odom`이 위치 예측을 제공합니다.
- `/imu`가 yaw를 안정화합니다.
- local scan matching이 각 scan 적재 전에 예측 pose를 미세 보정합니다.
- 그 보정 pose로 `map -> odom` TF도 발행해 RViz 정합성을 높입니다.

```mermaid
sequenceDiagram
    participant Odom as /odom
    participant IMU as /imu
    participant Scan as /scan
    participant Map as amr_map_server
    participant TF as map->odom TF
    participant Temp as /amr/map/temporary

    Odom->>Map: Predicted position
    IMU->>Map: Heading update
    Scan->>Map: Laser scan
    Map->>Map: Build predicted pose
    Map->>Map: Local scan matching
    Map->>Map: Raytrace free / occupied cells
    Map->>TF: Publish corrected map->odom
    Map->>Temp: Publish temporary map
```

### 3. Temporary Map Quality Evaluation and Freeze/Save

English:
- The temporary map is evaluated by coverage and planning-readiness metrics.
- Known/free/occupied ratios are checked.
- An inflation-aware free-space ratio is used as a lightweight planning feasibility check.
- If the map passes, it can be promoted to `/amr/map/data` and saved as `pgm + yaml`.
- The same flow can be triggered manually or automatically after consecutive successful evaluations.

한국어:
- temporary map은 coverage와 planning 가능성 기준으로 평가됩니다.
- known/free/occupied 비율을 점검합니다.
- inflation 이후 free-space 비율을 이용해 경로 생성 가능성을 가볍게 확인합니다.
- 기준을 통과하면 `/amr/map/data`로 승격하고 `pgm + yaml`로 저장할 수 있습니다.
- 이 흐름은 수동 service 또는 연속 통과 기반 자동 저장으로 동작할 수 있습니다.

```mermaid
sequenceDiagram
    participant Temp as /amr/map/temporary
    participant Map as amr_map_server
    participant Quality as quality evaluator
    participant Official as /amr/map/data
    participant Files as pgm + yaml

    Temp->>Map: Current temporary map
    Map->>Quality: Evaluate coverage and inflated free space
    Quality-->>Map: pass / fail
    alt passed
        Map->>Official: Promote temporary map
        Map->>Files: Save pgm + yaml
    else failed
        Map->>Temp: Continue mapping
    end
```

### 4. Frontier Exploration Roadmap

English:
- Frontier means the boundary between known space and unknown space.
- Exploration should choose frontier goals instead of random goals.
- The mapper keeps updating the temporary map.
- A frontier selector can choose the next reachable frontier cluster.
- The navigator can then drive the robot to that frontier until the map is complete.

한국어:
- frontier는 known 공간과 unknown 공간의 경계를 뜻합니다.
- exploration은 무작위 goal보다 frontier goal을 선택하는 쪽이 적합합니다.
- mapper가 temporary map을 계속 갱신합니다.
- frontier selector가 다음에 갈 수 있는 frontier cluster를 선택합니다.
- navigator는 그 frontier까지 주행시키며 맵 완성도를 올립니다.

```mermaid
sequenceDiagram
    participant Map as /amr/map/temporary
    participant Frontier as frontier selector
    participant Nav as amr_bt_navigator
    participant Global as amr_global_planner
    participant Local as amr_local_planner
    participant Motion as amr_motion_controller

    Map->>Frontier: Known / unknown boundary
    Frontier->>Frontier: Cluster frontiers
    Frontier-->>Nav: Next frontier goal
    Nav->>Global: Plan to frontier
    Global-->>Nav: Global path
    Nav->>Local: MotionCommand
    Local-->>Motion: Local path
    Motion-->>Nav: Progress / completion
```

## Bringup

Build the full stack:

```bash
colcon build --packages-up-to amr_navigation
```

Start map, localization, and global planner first:

```bash
ros2 launch amr_bringup localization.launch.py
```

Start navigation execution in another terminal:

```bash
ros2 launch amr_bringup navigation.launch.py
```

Shared runtime parameters live in [amr.yaml](./amr_bringup/params/amr.yaml).

## Interfaces

### Action

- `/amr/navigator/navigate_to_pose`
  - `amr_msgs/action/NavigateToPose`

### Services

- `/amr/map_server/get_map`
  - `nav_msgs/srv/GetMap`
- `/amr/map_server/freeze_temporary_map`
  - `std_srvs/srv/Trigger`
- `/amr/map_server/evaluate_temporary_map`
  - `std_srvs/srv/Trigger`
- `/amr/map_server/save_temporary_map`
  - `std_srvs/srv/Trigger`
- `/amr/global_planner/plan_segment`
  - `amr_msgs/srv/PlanSegment`
- `/amr/global_planner/plan_route`
  - `amr_msgs/srv/PlanRoute`

### Topics

- `/amr/map/data`
  - `nav_msgs/msg/OccupancyGrid`
- `/amr/map/temporary`
  - `nav_msgs/msg/OccupancyGrid`
- `/amr/costmap/global`
  - `nav_msgs/msg/OccupancyGrid`
- `/amr/costmap/local`
  - `nav_msgs/msg/OccupancyGrid`
- `/amr/localization/pose`
  - `geometry_msgs/msg/PoseStamped`
- `/amr/localization/odometry`
  - `nav_msgs/msg/Odometry`
- `/amr/planner/global`
  - `nav_msgs/msg/Path`
- `/amr/planner/local`
  - `nav_msgs/msg/Path`
- `/amr/motion/command`
  - `amr_msgs/msg/MotionCommand`
- `/amr/motion/status`
  - `amr_msgs/msg/MotionStatus`
- `/cmd_vel`
  - `geometry_msgs/msg/Twist`

## Notes

- the current stack is an MVP-focused custom navigation pipeline
- `amr_localization` is AMCL-lite rather than a full Nav2-compatible AMCL replacement
- both global and local planning now operate on inflated occupancy data
- RViz `2D Goal Pose` can be sent directly to the action server through `amr_rviz_plugins`
