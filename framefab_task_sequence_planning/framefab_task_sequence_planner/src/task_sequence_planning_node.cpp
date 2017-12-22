//
// Created by yijiangh on 12/21/17.
//

#include <ros/ros.h>

#include <framefab_task_sequence_planner/FiberPrintPlugIn.h>

// framefab dependencies
//#include "framefab_task_sequence_planning/utils/WireFrame.h"

// msg
#include <framefab_msgs/TaskSequencePlanning.h>

// Globals
const static std::string DEFAULT_TASK_SEQUENCE_PLANNING_SERVICE = "task_sequence_planning";

bool planTaskSequenceCallback(framefab_msgs::TaskSequencePlanningRequest& req,
                              framefab_msgs::TaskSequencePlanningResponse& res)
{
  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "framefab_task_sequence_planning");
  ros::NodeHandle nh;

  ros::ServiceServer task_sequence_planning_server =
      nh.advertiseService<framefab_msgs::TaskSequencePlanningRequest, framefab_msgs::TaskSequencePlanningResponse>(
          "task_sequence_planning", boost::bind(planTaskSequenceCallback, _1, _2));

  // Serve and wait for shutdown
  ROS_INFO_STREAM("[tsp] sequence task planning server online");
  ros::spin();

  return 0;
}