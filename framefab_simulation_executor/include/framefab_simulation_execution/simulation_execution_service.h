//
// Created by yijiangh on 7/12/17.
//

#ifndef FRAMEFAB_PATH_EXECUTION_PATH_EXECUTION_SERVICE_H
#define FRAMEFAB_PATH_EXECUTION_PATH_EXECUTION_SERVICE_H

#include <ros/ros.h>
#include <framefab_msgs/TrajectoryExecution.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

namespace framefab_simulation_execution
{

// this class takes "simulation_execution" action from framefab_execution_gatekeeper
// and generate "joint_trajectory_action" for simulation
// for visualization in Choreo, we use industrial_robot_simulator as action server.

class SimulationExecutionService
{
 public:
  SimulationExecutionService(ros::NodeHandle& nh);

  bool executionCallback(framefab_msgs::TrajectoryExecution::Request& req,
                         framefab_msgs::TrajectoryExecution::Response& res);

 private:
  ros::ServiceServer server_;
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac_;
  std::string name_;
};
}

#endif //FRAMEFAB_PATH_EXECUTION_PATH_EXECUTION_SERVICE_H
