//
// Created by yijiangh on 7/10/17.
//

#include <framefab_execution_gatekeeper/framefab_execution_gatekeeper_service.h>

#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "framefab_execution_gatekeeper_service_node");

  ros::NodeHandle nh;

  framefab_execution_gatekeeper::ExecutionGatekeeper plan_executor(nh);

  ros::spin();
  return 0;
}