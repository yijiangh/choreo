//
// Created by yijiangh on 7/10/17.
//

#include <framefab_process_execution/framefab_process_execution_service.h>

#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "framefab_process_execution_service_node");

  ros::NodeHandle nh;

  framefab_process_execution::FrameFabProcessExecutionService process_executor(nh);

  ros::spin();
  return 0;
}