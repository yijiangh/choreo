# process_type primitives
int32 TRANSITION=2
int32 NEAR_MODEL=1
int32 PROCESS=0

# main_data_type primitives
int32 JOINT=-1
int32 CART=-2

# hosting unit_process's id
int32 unit_process_id

# id (order) in the subprocess list under the host
int32 sub_process_id

# process type, see above
int32 process_type

# main data type, see above
int32 main_data_type

# joint_array
trajectory_msgs/JointTrajectory joint_array

# end effector cartesian pose array
geometry_msgs/Pose[] TCP_pose_array

# comment
string comment
