# @brief Command dispatched by the navigator or recovery server to the motion controller.

# @brief Follow a navigation path toward goal_pose.
uint8 MODE_NAVIGATE=0
# @brief Execute a reverse recovery motion.
uint8 MODE_BACKUP=1
# @brief Execute an in-place spin recovery motion.
uint8 MODE_SPIN=2
# @brief Hold position for recovery_duration.
uint8 MODE_WAIT=3

# @brief Message timestamp and frame metadata.
std_msgs/Header header
# @brief Monotonic command identifier used to match status feedback.
uint32 command_id
# @brief Command mode, one of MODE_* constants.
uint8 mode
# @brief Route identifier propagated through logs and status.
string route_id
# @brief Behavior-tree or waypoint node identifier.
string node_id
# @brief Path to track for MODE_NAVIGATE.
nav_msgs/Path plan
# @brief Final goal pose for tracking and goal checking.
geometry_msgs/PoseStamped goal_pose
# @brief True when the controller should align yaw at the goal.
bool align_heading_at_goal
# @brief Backup distance for MODE_BACKUP.
float64 recovery_distance
# @brief Linear speed for recovery motion.
float64 recovery_speed
# @brief Spin angle for MODE_SPIN.
float64 recovery_angle
# @brief Duration for MODE_WAIT or bounded recovery execution.
float64 recovery_duration
