//
// Created by yijiangh on 7/12/17.
//

#include <framefab_path_execution/path_execution_service.h>

#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "path_execution_service_node");

  ros::NodeHandle nh;

  framefab_path_execution::PathExecutionService path_executor(nh);

  ros::spin();
  return 0;
}