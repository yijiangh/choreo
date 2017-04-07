#include <ros/ros.h>

class kuka_node
{
public:
kuka_node() {}
~kuka_node() {}

public:
void setNodeHandle(ros::NodeHandle* _ptr_node_handle) { ptr_node_handle_ = _ptr_node_handle; }

// call back function for message subscription
void mplanCallback(std_msgs::Bool mp_msg);

private:
ros::NodeHandle* ptr_node_handle_;
}
