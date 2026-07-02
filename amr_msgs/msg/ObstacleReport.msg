# @brief Obstacle observation summary for navigation and diagnostics.

# @brief Message timestamp and frame metadata.
std_msgs/Header header

# @brief No obstacle severity.
uint8 SEVERITY_NONE=0
# @brief Low obstacle severity.
uint8 SEVERITY_LOW=1
# @brief Medium obstacle severity.
uint8 SEVERITY_MEDIUM=2
# @brief High obstacle severity.
uint8 SEVERITY_HIGH=3
# @brief Critical obstacle severity.
uint8 SEVERITY_CRITICAL=4

# @brief True when this report describes an active obstacle.
bool active
# @brief True when obstacle was detected as dynamic.
bool is_dynamic
# @brief True when obstacle blocks the planned path.
bool blocks_path
# @brief Severity code, one of SEVERITY_* constants.
uint8 severity
# @brief Obstacle distance in meters.
float64 distance
# @brief Obstacle bearing in radians.
float64 bearing
# @brief Estimated obstacle point.
geometry_msgs/Point obstacle_point
# @brief Source subsystem that generated the report.
string source
