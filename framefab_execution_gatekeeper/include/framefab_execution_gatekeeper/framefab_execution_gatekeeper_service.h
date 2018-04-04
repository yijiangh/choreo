//
// Created by yijiangh on 7/10/17.
//

#ifndef FRAMEFAB_SIMULATION_GATEKEEPER_H
#define FRAMEFAB_SIMULATION_GATEKEEPER_H

#include <ros/ros.h>
#include <framefab_msgs/ProcessExecutionAction.h>
#include <actionlib/server/simple_action_server.h>

namespace framefab_execution_gatekeeper
{

// this class is the action server for the ProcessExecutionAction call from core service node
// It acts like a gatekeeper, directing the trajectory to different execution instances
// for now, it only supports directing trajectory to simulator (framefab_simulation_execution).
class ExecutionGatekeeper
{
 public:
  ExecutionGatekeeper(ros::NodeHandle& nh);

  void executionCallback(const framefab_msgs::ProcessExecutionGoalConstPtr &goal);
  bool simulateProcess(const framefab_msgs::ProcessExecutionGoalConstPtr &goal);

  // TODO: execution port to be implemented in the future
//  bool executeProcess(const framefab_msgs::ProcessExecutionGoalConstPtr &goal);

 private:
  ros::NodeHandle nh_;
//  ros::ServiceClient real_client_;
  ros::ServiceClient sim_client_;
  actionlib::SimpleActionServer<framefab_msgs::ProcessExecutionAction> process_exe_action_server_;
};
}

#endif //FRAMEFAB_SIMULATION_GATEKEEPER_H