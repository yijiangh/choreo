//
// Created by yijiangh on 7/10/17.
//

#include <framefab_process_execution/framefab_process_execution_service.h>

#include <framefab_industrial_robot_simulator_service/SimulateTrajectory.h>
#include <framefab_msgs/TrajectoryExecution.h>

#include "process_utils.h"
#include <boost/thread.hpp>

#include <ros/topic.h>

const static std::string EXECUTION_SERVICE_NAME = "path_execution";
const static std::string SIMULATION_SERVICE_NAME = "process_simulation";
const static std::string PROCESS_EXE_ACTION_SERVER_NAME = "framefab_process_execution_as";

framefab_process_execution::FrameFabProcessExecutionService::FrameFabProcessExecutionService(ros::NodeHandle& nh)
    : nh_(nh),
      process_exe_action_server_(nh_,
                                 PROCESS_EXE_ACTION_SERVER_NAME,
                                 boost::bind(&framefab_process_execution::FrameFabProcessExecutionService::executionCallback, this, _1),
                                 false)
{
  // Simulation Server
  sim_client_ = nh_.serviceClient<framefab_industrial_robot_simulator_service::SimulateTrajectory>(
      SIMULATION_SERVICE_NAME);

  // Trajectory Execution Service
  real_client_ = nh_.serviceClient<framefab_msgs::TrajectoryExecution>(EXECUTION_SERVICE_NAME);

  // The generic process execution service
  process_exe_action_server_.start();
}

void framefab_process_execution::FrameFabProcessExecutionService::executionCallback(
    const framefab_msgs::ProcessExecutionGoalConstPtr &goal)
{
  framefab_msgs::ProcessExecutionResult res;
  if (goal->simulate)
  {
    res.success = simulateProcess(goal);
  }
  else
  {
    res.success = executeProcess(goal);
  }
  process_exe_action_server_.setSucceeded(res);
}

bool framefab_process_execution::FrameFabProcessExecutionService::executeProcess(
    const framefab_msgs::ProcessExecutionGoalConstPtr &goal)
{
  framefab_msgs::TrajectoryExecution exe_srv;
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

  if (!real_client_.call(exe_srv))
  {
    ROS_ERROR("Execution client unavailable or unable to execute trajectory.");
    return false;
  }

  return true;
}

bool framefab_process_execution::FrameFabProcessExecutionService::simulateProcess(
    const framefab_msgs::ProcessExecutionGoalConstPtr &goal)
{
  using framefab_industrial_robot_simulator_service::SimulateTrajectory;

  // The simulation server doesn't support any I/O visualizations, so we aggregate the
  // trajectory components and send them all at once

  // Pass the trajectory to the simulation service
  SimulateTrajectory sim_srv;
  sim_srv.request.wait_for_execution = goal->wait_for_execution;

  for(auto jts : goal->joint_traj_array)
  {
    ROS_INFO_STREAM("[Simulate Traj] joint_traj - " << jts.points.size());

    if (sim_srv.request.trajectory.points.empty())
    {
      sim_srv.request.trajectory = jts;
    }
    else
    {
      appendTrajectory(sim_srv.request.trajectory, jts);
    }
  }

  // Call simulation service
  if (!sim_client_.call(sim_srv))
  {
    ROS_ERROR("[process execution] Simulation client unavailable or unable to simulate trajectory.");
    return false;
  }
  else
  {
    return true;
  }
}
