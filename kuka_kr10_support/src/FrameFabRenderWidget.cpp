//
// Created by yijiangh on 4/13/17.
//

#include <FrameFabRenderWidget.h>

namespace framefab {
    FrameFabRenderWidget::FrameFabRenderWidget(ros::NodeHandle &node_handle)
    : node_handle_(node_handle)
    {
        ROS_INFO("FrameFabRenderWidget class started.");

        // readParameters
        readParameters();

        // advertise topics - should be done in computation class
        display_marker_publisher_ = node_handle.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    }

    FrameFabRenderWidget::~FrameFabRenderWidget()
    {
    }

    bool FrameFabRenderWidget::readParameters()
    {
        // FrameFab Parameters
        node_handle_.param("display_pose_topic", display_pose_topic_, std::string("/framelinks"));
        node_handle_.param("read_file_topic", read_file_topic_, std::string("/readfile"));

        return true;
    }
}