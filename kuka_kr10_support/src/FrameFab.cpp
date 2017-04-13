#include <framefab/FrameFab.h>

namespace framefab{
    Framefab::Framefab(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle),
      ffMotionPlanner_(nodeHandle)
    {
        ROS_INFO("FrameFab node started.");

        // readParameters

        // advertise topics
        //todo: the topic queue size (1000, 1, etc) is only for temporal usage, not safe
        publisher_pose_ = nodeHandle_.advertise<geometry_msgs::PoseArray>("/framelinks", 1000);
        publisher_motionPlan_ = nodeHandle_.advertise<std_msgs::Bool>("/activate_mplan", 1);
    }

    Framefab::~Framefab()
    {
        nodeHandle_.shutdown();
    }

    Framefab::readParameters()
    {
    }

    Framefab::displayFrameCallback()
    {

    }
}
