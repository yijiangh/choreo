#include <FrameFab.h>

namespace framefab{
    FrameFab::FrameFab(ros::NodeHandle& node_handle)
    : node_handle_(node_handle)
    {
        ROS_INFO("FrameFab node started.");

        // readParameters
        readParameters();

        // advertise topics - should be done in computation class
//        publisher_pose_     = nodeHandle_.advertise<geometry_msgs::PoseArray>("/framelinks", 1);
//        publisher_mplan_    = nodeHandle_.advertise<std_msgs::Bool>("/activate_mplan", 1);

        // visualize framelinks message subsriber
        ros::Subscriber frame_sub = node_handle.subscribe(displaypose_topic_, 0, &frameCallback);

    }

    FrameFab::~FrameFab()
    {
        nodeHandle_.shutdown();
    }

    FrameFab::readParameters()
    {
        //FrameFab Parameters
        node_handle_.param("display_pose_topic", display_pose_topic_, string("/framelinks"));

        return true;
    }

    FrameFab::displayFrameCallback()
    {

    }
}
