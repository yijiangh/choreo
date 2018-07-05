# Enumeration represeting plan type
# 
int32 SUPPORT_TYPE=2
int32 CREATE_TYPE=1
int32 CONNECT_TYPE=0

int32 CONTINUOUS_TYPE=-1
int32 DISCONTINUOUS_TYPE=-2

# The plan is broken into components so that nodes responsible
# for the actual execution can perform any necessary IO commands
# as the robot begins and completes the actual process path. In
# choreo we export the tagged trajectory to graphical programming
# platform (e.g. grasshopper on Rhino) for fine-tuned IO weaving 
# process.

# process type
int32 type

# sub_processes
choreo_msgs/SubProcess[] sub_process_array
