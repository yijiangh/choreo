int32 element_id

# Enumeration element type
int32 SUPPORT=2
int32 CREATE=1
int32 CONNECT=0

int32 type

geometry_msgs/Point start_pt
geometry_msgs/Point end_pt

# shrinked collision objects

geometry_msgs/Vector3[] feasible_orients
