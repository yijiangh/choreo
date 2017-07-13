//
// Created by yijiangh on 7/12/17.
//

#ifndef FRAMEFAB_PATH_EXECUTION_PATH_EXECUTION_SERVICE_H
#define FRAMEFAB_PATH_EXECUTION_PATH_EXECUTION_SERVICE_H

#include <ros/ros.h>
#include <framefab_msgs/TrajectoryExecution.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

namespace framefab_path_execution
{

class PathExecutionService
{
 public:
  PathExecutionService(ros::NodeHandle& nh);

  /**
   * Currently forwards the framefab_msgs::TrajectoryExecution on to the corresponding
   * MoveIt node. The idea though is that abstracting 'execution' will give us more flexibility
   * later to implement our own process parameters related to execution.
   */
  bool executionCallback(framefab_msgs::TrajectoryExecution::Request& req,
                         framefab_msgs::TrajectoryExecution::Response& res);

 private:
  ros::ServiceServer server_;
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac_;
  std::string name_;
};
}

#endif //FRAMEFAB_PATH_EXECUTION_PATH_EXECUTION_SERVICE_H
