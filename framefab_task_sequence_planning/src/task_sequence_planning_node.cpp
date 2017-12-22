//
// Created by yijiangh on 12/21/17.
//

#include <ros/ros.h>
#include <framefab_task_sequence_planning/framefab_task_sequence_planning.h>

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

  ros::ServiceServer task_sequence_processing_server =
      nh.advertiseService<framefab_msgs::TaskSequenceProcessingRequest, framefab_msgs::TaskSequenceProcessingResponse>(
          "task_sequence_processing", boost::bind(processTaskSequenceCallback, _1, _2));

  // Serve and wait for shutdown
  ROS_INFO_STREAM("[tsp] sequence task planning server online");
  ros::spin();

  return 0;
}