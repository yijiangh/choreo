//
// Created by yijiangh on 7/10/17.
//

#include <choreo_execution_gatekeeper/choreo_execution_gatekeeper_service.h>

#include <choreo_msgs/TrajectoryExecution.h>

#include "execution_utils.h"
#include <boost/thread.hpp>

#include <ros/topic.h>

const static std::string SIMULATION_SERVICE_NAME = "simulation_execution";
const static std::string PROCESS_EXE_ACTION_SERVER_NAME = "choreo_execution_as";

choreo_execution_gatekeeper::ExecutionGatekeeper::ExecutionGatekeeper(ros::NodeHandle& nh)
    : nh_(nh),
      process_exe_action_server_(nh_,
                                 PROCESS_EXE_ACTION_SERVER_NAME,
                                 boost::bind(&choreo_execution_gatekeeper::ExecutionGatekeeper::executionCallback, this, _1),
                                 false)
{
  // Trajectory Simulation Service
  sim_client_ = nh_.serviceClient<choreo_msgs::TrajectoryExecution>(SIMULATION_SERVICE_NAME);

  // The generic process execution service
  process_exe_action_server_.start();
}

void choreo_execution_gatekeeper::ExecutionGatekeeper::executionCallback(
    const choreo_msgs::ProcessExecutionGoalConstPtr &goal)
{
  choreo_msgs::ProcessExecutionResult res;
  if (goal->simulate)
  {
    res.success = simulateProcess(goal);
  }

  process_exe_action_server_.setSucceeded(res);
}

bool choreo_execution_gatekeeper::ExecutionGatekeeper::simulateProcess(
    const choreo_msgs::ProcessExecutionGoalConstPtr &goal)
{
  choreo_msgs::TrajectoryExecution exe_srv;
  exe_srv.request.wait_for_execution = true;

  for(auto jts : goal->joint_traj_array)
  {
    if (exe_srv.request.trajectory.points.empty())
    {
      exe_srv.request.trajectory = jts;
    }
    else
    {
      appendTrajectory(exe_srv.request.trajectory, jts);
    }
  }

  // this action call is directed to choreo_simulation_execution
  if (!sim_client_.call(exe_srv))
  {
    ROS_ERROR("[Execution GateKeeper] Execution client unavailable or unable to execute trajectory.");
    return false;
  }

  return true;
}