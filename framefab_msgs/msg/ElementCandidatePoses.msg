int32 element_id

# Enumeration element type
int32 SUPPORT=2
int32 CREATE=1
int32 CONNECT=0

int32 type

geometry_msgs/Point start_pt
geometry_msgs/Point end_pt

float64 element_diameter

geometry_msgs/Vector3[] feasible_orients

# shirnked end pt for collision checking
geometry_msgs/Point shrinked_start_pt
geometry_msgs/Point shrinked_end_pt

### As the geoemtry being processed should not
### be limited to "cylinder" shape that's related
### to current development targeted at linear element
### Thus, the shrinked geometry is temporarily placed in ### this msg as a reminder.
 
# both sides shrinked collision objects
moveit_msgs/CollisionObject both_side_shrinked_collision_object

# start side shrinked geometry
moveit_msgs/CollisionObject st_shrinked_collision_object

# end side shrinked geometry
moveit_msgs/CollisionObject end_shrinked_collision_object

# full geometry
moveit_msgs/CollisionObject full_collision_object
