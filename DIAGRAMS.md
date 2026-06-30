# Architecture Diagrams

## Online SLAM Navigation

```mermaid
flowchart LR
    Robot["robot bringup"] --> Scan["/scan"]
    Robot --> OdomTf["odom -> base_* TF"]
    Robot --> Tf["/tf, /tf_static"]

    Scan --> Slam["external slam_toolbox online_async_launch.py"]
    OdomTf --> Slam
    Tf --> Slam

    Slam --> Map["/map"]
    Slam --> MapOdom["map -> odom TF"]

    Map --> Costmap["amr_costmap_server"]
    Scan --> Costmap
    Tf --> Costmap
    MapOdom --> Costmap

    Costmap --> GlobalCostmap["/global_costmap"]
    Costmap --> LocalCostmap["/local_costmap"]

    GlobalCostmap --> GlobalPlanner["amr_global_planner"]
    GlobalPlanner --> GlobalPlan["/global_plan"]
    GlobalPlanner --> PlanServices["/plan_segment, /plan_route"]

    PlanServices --> Navigator["amr_bt_navigator"]
    Tf --> Navigator
    MapOdom --> Navigator
    Navigator --> MotionCommand["/motion_command"]
    Navigator --> NavActions["/navigate_to_pose, /navigate_to_poses"]

    MotionCommand --> LocalPlanner["local_planner"]
    GlobalPlan --> LocalPlanner
    LocalCostmap --> LocalPlanner
    Tf --> LocalPlanner
    MapOdom --> LocalPlanner
    LocalPlanner --> LocalPlan["/local_plan"]
    LocalPlanner --> LocalPlanStatus["/local_plan_status"]

    MotionCommand --> MotionController["motion_controller"]
    LocalPlan --> MotionController
    Scan --> MotionController
    Tf --> MotionController
    MapOdom --> MotionController
    MotionController --> CmdVel["/cmd_vel"]
    MotionController --> MotionStatus["/motion_status"]

    MotionStatus --> Navigator
    LocalPlanStatus --> Navigator
    Navigator --> RecoveryServices["/plan_recovery, /plan_local_escape, /clear_costmap"]
```

## TF Ownership

```mermaid
flowchart TD
    SlamToolbox["external slam_toolbox"] --> MapOdom["publishes map -> odom"]
    RobotBringup["robot bringup"] --> OdomBase["publishes odom -> base_footprint or odom -> base_link"]
    AMR["ros-amr-navigation"] --> Consume["consumes map -> odom -> base_*"]
    MapOdom --> Consume
    OdomBase --> Consume
    AMR -. does not publish .-> MapOdom
```

The AMR navigation launch must not start any node that publishes `map -> odom`.
`amr_localization` is therefore not part of the navigation core.

## Launch Sequence

```mermaid
sequenceDiagram
    participant Robot as robot bringup
    participant Slam as external slam_toolbox online_async
    participant Nav as amr_bringup navigation.launch.py

    Robot->>Robot: publish /scan and odom -> base_*
    Slam->>Slam: consume sensors and TF with packaged AMR params
    Slam->>Nav: publish /map
    Slam->>Nav: publish map -> odom TF
    Nav->>Nav: lifecycle configure/activate costmap, planners, controller, navigator, recovery
    Nav->>Robot: publish /cmd_vel
```

## Navigation Core Lifecycle

```mermaid
flowchart LR
    Manager["/navigation_manager"] --> Costmap["/costmap_server"]
    Manager --> Global["/global_planner"]
    Manager --> Local["/local_planner"]
    Manager --> Motion["/motion_controller"]
    Manager --> Recovery["/recovery_server"]
    Manager --> Navigator["/navigator"]
```

`/navigation_manager` manages only navigation core lifecycle nodes. It does not
publish an initial pose by default; `/initialpose` remains a `slam_toolbox`/RViz
interface.

The lifecycle nodes are launched in the root namespace. Their managed names are
`/costmap_server`, `/global_planner`, `/local_planner`, `/motion_controller`,
`/recovery_server`, and `/navigator`.
