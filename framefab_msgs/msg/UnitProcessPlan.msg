# Enumeration represeting plan type  
int32 SUPPORT_TYPE=2
int32 CONTINUOUS_TYPE=1
int32 DISCONTINUOUS_TYPE=0

# The plan is broken into components so that nodes responsible
# for the actual execution can perform any necessary IO commands
# as the robot begins and completes the actual process path. In
# framefab we export the tagged trajectory to graphical programming
# platform (e.g. grasshopper on Rhino) for fine-tuned IO weaving 
# process.

int32 type

trajectory_msgs/JointTrajectory trajectory_connection
trajectory_msgs/JointTrajectory trajectory_approach
trajectory_msgs/JointTrajectory trajectory_process
trajectory_msgs/JointTrajectory trajectory_depart

geometry_msgs/Pose near_start
geometry_msgs/Pose start
geometry_msgs/Pose end
geometry_msgs/Pose near_end
