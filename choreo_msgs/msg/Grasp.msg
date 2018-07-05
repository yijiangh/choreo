# This message contains a description of a grasp that would be used
# with a particular end-effector to grasp an object, including how to
# approach it, grip it, etc. Note that this message does not have any info
# about the "grasp point", aka the relationship between the EE and the object
# It only has info about the pose of a "floating" end effector.

geometry_msgs/Pose pick_grasp_pose
geometry_msgs/Pose pick_grasp_approach_pose
geometry_msgs/Pose pick_grasp_retreat_pose

geometry_msgs/Pose place_grasp_pose
geometry_msgs/Pose place_grasp_approach_pose
geometry_msgs/Pose place_grasp_retreat_pose