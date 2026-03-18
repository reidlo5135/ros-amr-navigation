std_msgs/Header header

uint8 SEVERITY_NONE=0
uint8 SEVERITY_LOW=1
uint8 SEVERITY_MEDIUM=2
uint8 SEVERITY_HIGH=3
uint8 SEVERITY_CRITICAL=4

bool active
bool is_dynamic
bool blocks_path
uint8 severity
float64 distance
float64 bearing
geometry_msgs/Point obstacle_point
string source
