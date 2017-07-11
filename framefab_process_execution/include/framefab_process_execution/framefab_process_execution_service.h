//
// Created by yijiangh on 7/10/17.
//

#ifndef FRAMEFAB_PROCESS_EXECUTION_FRAMEFAB_PROCESS_SERVICE_H
#define FRAMEFAB_PROCESS_EXECUTION_FRAMEFAB_PROCESS_SERVICE_H

#include <ros/ros.h>
#include <framefab_msgs/ProcessExecutionAction.h>
#include <actionlib/server/simple_action_server.h>

namespace framefab_process_execution
{

class FrameFabProcessExecutionService
{
 public:
  FrameFabProcessExecutionService(ros::NodeHandle& nh);

  void executionCallback(const framefab_msgs::ProcessExecutionGoalConstPtr &goal);
  bool executeProcess(const framefab_msgs::ProcessExecutionGoalConstPtr &goal);
  bool simulateProcess(const framefab_msgs::ProcessExecutionGoalConstPtr &goal);

 private:
  ros::NodeHandle nh_;
  ros::ServiceClient real_client_;
  ros::ServiceClient sim_client_;
  actionlib::SimpleActionServer<framefab_msgs::ProcessExecutionAction> process_exe_action_server_;
  bool j23_coupled_;
};
}

#endif //FRAMEFAB_PROCESS_EXECUTION_FRAMEFAB_PROCESS_SERVICE_H