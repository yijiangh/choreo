#ifndef KUKA_FRAMEFAB_PROCESS_SERVICE_H
#define KUKA_FRAMEFAB_PROCESS_SERVICE_H

#include <ros/ros.h>
#include <framefab_msgs/ProcessExecutionAction.h>
#include <actionlib/server/simple_action_server.h>

namespace framefab_process_execution
{

class KukaFrameFabProcessService
{
public:
  KukaframefabProcessService(ros::NodeHandle& nh);

  /**
   * Currently forwards the framefab_msgs::TrajectoryExecution on to the corresponding
   * MoveIt node. The idea though is that abstracting 'execution' will give us more flexibility
   * later to implement our own process parameters related to execution.
   */
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

#endif // kuka_FRAMEFAB_PROCESS_SERVICE_H
