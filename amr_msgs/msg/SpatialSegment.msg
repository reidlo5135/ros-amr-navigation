uint8 UNKNOWN=0
uint8 OPEN_AREA=1
uint8 CORRIDOR=2
uint8 JUNCTION=3
uint8 DOOR_CANDIDATE=4
uint8 NARROW_PASSAGE=5

uint32 id
uint8 type
float32 confidence

geometry_msgs/Pose centroid
geometry_msgs/Point[] polygon
geometry_msgs/Point[] centerline

geometry_msgs/Point bbox_min
geometry_msgs/Point bbox_max

float32 area_m2
float32 length_m
float32 width_m
