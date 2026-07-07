# @brief Frontier navigation status for visualization overlays.
std_msgs/Header header
# @brief High-level phase label.
string phase
# @brief User-requested original goal in the map frame.
geometry_msgs/PoseStamped original_goal
# @brief Active known/free original or staging goal.
geometry_msgs/PoseStamped active_known_goal
# @brief Current staging iteration.
uint16 iteration
# @brief True while /navigate_to_unknown_pose is executing.
bool active
# @brief True when the latest map says the original goal is known/free.
bool original_goal_known
# @brief Euclidean distance from current pose to the original goal.
float32 distance_to_original_goal
# @brief Human-readable detail for UI/event panes.
string message
