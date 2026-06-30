# amr_mqtt_server

Optional MQTT bridge for the AMR navigation stack.

The ROS-side defaults follow the current navigation contract:

- `/map`
- `/global_plan`
- `/local_plan`
- `/global_costmap`
- `/local_costmap`
- `/motion_status`
- `/cmd_vel`
- `/initialpose`
- `/plan_segment`
- `/plan_route`
- `/navigate_to_poses`

`robot_pose` is an optional visualization input and defaults to `/pose`; it is
not required by the navigation core.

The MQTT broker topic prefix is an application protocol setting and is separate
from ROS topic names. Configure it under `mqtt.topics.header` in
`config/amr_mqtt_server.yaml`; the default broker root is `/navigation`.

Default launch:

```bash
ros2 launch amr_mqtt_server amr_mqtt_server.launch.py
```

Or start it with navigation:

```bash
ros2 launch amr_bringup navigation.launch.py use_mqtt_server:=true
```
