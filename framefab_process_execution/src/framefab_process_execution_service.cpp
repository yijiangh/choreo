//
// Created by yijiangh on 7/10/17.
//

#include <framefab_process_execution/framefab_process_execution_service.h>

#include <industrial_robot_simulator_service/SimulateTrajectory.h>
#include <framefab_msgs/TrajectoryExecution.h>

#include "process_utils.h"
#include <boost/thread.hpp>

#include <ros/topic.h>

const static std::string EXECUTION_SERVICE_NAME = "process_execution";
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
  sim_client_ = nh_.serviceClient<industrial_robot_simulator_service::SimulateTrajectory>(
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
  framefab_msgs::TrajectoryExecution srv_connection;
  srv_connection.request.wait_for_execution = true;
  srv_connection.request.trajectory = goal->trajectory_connection;

  framefab_msgs::TrajectoryExecution srv_approach;
  srv_approach.request.wait_for_execution = true;
  srv_approach.request.trajectory = goal->trajectory_approach;

  framefab_msgs::TrajectoryExecution srv_process;
  srv_process.request.wait_for_execution = true;
  srv_process.request.trajectory = goal->trajectory_process;

  framefab_msgs::TrajectoryExecution srv_depart;
  srv_depart.request.wait_for_execution = true;
  srv_depart.request.trajectory = goal->trajectory_depart;

  if (!real_client_.call(srv_connection))
  {
    ROS_ERROR("Execution client unavailable or unable to execute connection trajectory.");
    return false;
  }

  if (!real_client_.call(srv_approach))
  {
    ROS_ERROR("Execution client unavailable or unable to execute approach trajectory.");
    return false;
  }

  if (!real_client_.call(srv_process))
  {
    ROS_ERROR("Execution client unavailable or unable to execute process trajectory.");
    return false;
  }

  if (!real_client_.call(srv_depart))
  {
    ROS_ERROR("Execution client unavailable or unable to execute departure trajectory.");
    return false;
  }

  return true;
}

bool framefab_process_execution::FrameFabProcessExecutionService::simulateProcess(
    const framefab_msgs::ProcessExecutionGoalConstPtr &goal)
{
  using industrial_robot_simulator_service::SimulateTrajectory;

  // The simulation server doesn't support any I/O visualizations, so we aggregate the
  // trajectory components and send them all at once
  trajectory_msgs::JointTrajectory aggregate_traj;
  aggregate_traj = goal->trajectory_connection;
  appendTrajectory(aggregate_traj, goal->trajectory_approach);
  appendTrajectory(aggregate_traj, goal->trajectory_process);
  appendTrajectory(aggregate_traj, goal->trajectory_depart);

  // Pass the trajectory to the simulation service
  SimulateTrajectory srv;
  srv.request.wait_for_execution = goal->wait_for_execution;
  srv.request.trajectory = aggregate_traj;

  // Call simulation service
  if (!sim_client_.call(srv))
  {
    ROS_ERROR("[process execution] Simulation client unavailable or unable to simulate trajectory.");
    return false;
  }
  else
  {
    return true;
  }
}