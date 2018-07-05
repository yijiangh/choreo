//
// Created by yijiangh on 7/12/17.
//

#ifndef CHOREO_PATH_EXECUTION_PATH_EXECUTION_SERVICE_H
#define CHOREO_PATH_EXECUTION_PATH_EXECUTION_SERVICE_H

#include <ros/ros.h>
#include <choreo_msgs/TrajectoryExecution.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

namespace choreo_simulation_execution
{

// this class takes "simulation_execution" action from choreo_execution_gatekeeper
// and generate "joint_trajectory_action" for simulation
// for visualization in Choreo, we use industrial_robot_simulator as action server.

class SimulationExecutionService
{
 public:
  SimulationExecutionService(ros::NodeHandle& nh);

  bool executionCallback(choreo_msgs::TrajectoryExecution::Request& req,
                         choreo_msgs::TrajectoryExecution::Response& res);

 private:
  ros::ServiceServer server_;
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac_;
  std::string name_;
};
}

#endif //CHOREO_PATH_EXECUTION_PATH_EXECUTION_SERVICE_H
