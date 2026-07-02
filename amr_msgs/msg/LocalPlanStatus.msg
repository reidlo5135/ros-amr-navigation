# @brief Local planner status and recovery decision output.

# @brief Message timestamp and frame metadata.
std_msgs/Header header
# @brief Associated motion command id.
uint32 command_id
# @brief True when local planning is active.
bool active
# @brief True when the published local plan is valid.
bool local_plan_valid
# @brief True when recovery or replanning is required.
bool recovery_required
# @brief Local planner reports normal operation.
uint8 DECISION_OK=0
# @brief Goal-proximity obstruction requires special handling.
uint8 DECISION_GOAL_PROXIMITY_BLOCKED=1
# @brief Global replan is required.
uint8 DECISION_GLOBAL_REPLAN_REQUIRED=2
# @brief Hard blockage requires recovery.
uint8 DECISION_HARD_BLOCKED=3
# @brief Decision code, one of DECISION_* constants.
uint8 decision
# @brief True when blocked_pose contains a valid pose.
bool has_blocked_pose
# @brief First blocked pose found on the plan.
geometry_msgs/PoseStamped blocked_pose
# @brief Distance from current pose to blocked pose.
float64 blocked_distance
