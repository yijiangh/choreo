//
// Created by yijiangh on 7/12/17.
//

#include <framefab_simulation_execution/simulation_execution_service.h>
#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "simulation_execution_service_node");

  ros::NodeHandle nh;

  framefab_simulation_execution::SimulationExecutionService path_executor(nh);

  ros::spin();
  return 0;
}