# amr_viz

React-based MQTT visualization client for the AMR stack.

## Role

- connects directly to the MQTT broker over WebSocket
- does not depend on ROS runtime on the operator machine
- visualizes:
  - raw SLAM map
  - static global costmap
  - dynamic local costmap
  - robot pose and TF
  - global and local plans
  - LaserScan
  - goal and initial pose markers

## MQTT Topics

Telemetry:
- `amr/robot/turtlebot3/viz/map`
- `amr/robot/turtlebot3/viz/global_costmap`
- `amr/robot/turtlebot3/viz/local_costmap`
- `amr/robot/turtlebot3/viz/robot_pose`
- `amr/robot/turtlebot3/viz/global_path`
- `amr/robot/turtlebot3/viz/local_path`
- `amr/robot/turtlebot3/viz/motion_status`
- `amr/robot/turtlebot3/viz/scan`
- `amr/robot/turtlebot3/viz/tf`
- `amr/robot/turtlebot3/viz/tf_static`
- `amr/robot/turtlebot3/viz/robot_description`

Commands:
- `amr/command/navigate_to_pose`
- `amr/command/cancel_navigate_to_pose`
- `amr/command/set_initial_pose`

## Interaction

- direct input of `x / y / yaw`
- map drag placement for goal and initial pose
- layer toggles for map, costmaps, robot, plans, scan, and TF
- OrbitControls for pan / rotate / zoom

## Run

```bash
npm install
npm run dev
```

Default browser MQTT endpoint:

```text
ws://<broker-host>:9001/mqtt
```
