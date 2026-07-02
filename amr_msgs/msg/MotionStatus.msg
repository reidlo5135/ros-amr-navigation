# @brief Motion controller status published for navigator and operator feedback.

# @brief Message timestamp and frame metadata.
std_msgs/Header header
# @brief Command id associated with this status.
uint32 command_id
# @brief Active motion mode.
uint8 mode
# @brief True when a command is currently active.
bool active
# @brief True when the active command has completed.
bool command_completed
# @brief True when the navigation goal has been reached.
bool goal_reached
# @brief True when obstacle context is detected.
bool obstacle_detected
# @brief True when the controller has latched blocked semantics.
bool blocked
# @brief True when progress checker reports stalled motion.
bool stalled
# @brief True when the current local plan is valid.
bool local_plan_valid
# @brief True when blockage came from costmap context.
bool costmap_blocked
# @brief True when the scan safety gate is blocking forward motion.
bool safety_gate_blocked
# @brief True when blocked_pose contains a valid pose.
bool has_blocked_pose
# @brief Current robot pose used by the controller.
geometry_msgs/PoseStamped current_pose
# @brief Pose associated with detected blockage.
geometry_msgs/PoseStamped blocked_pose
# @brief Remaining distance estimate in meters.
float64 remaining_distance
# @brief Current heading error in radians.
float64 heading_error
