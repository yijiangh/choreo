//
// Created by yijiangh on 7/10/17.
//

#include <choreo_execution_gatekeeper/choreo_execution_gatekeeper_service.h>
#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "choreo_execution_gatekeeper_service_node");

  ros::NodeHandle nh;

  choreo_execution_gatekeeper::ExecutionGatekeeper plan_executor(nh);

  ros::spin();
  return 0;
}