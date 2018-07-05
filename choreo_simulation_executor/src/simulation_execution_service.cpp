//
// Created by yijiangh on 7/12/17.
//

#include <choreo_simulation_execution/simulation_execution_service.h>

const static double ACTION_EXTRA_WAIT_RATIO = 2.0;   // 20% past end of trajectory
const static double ACTION_SERVICE_WAIT_TIME = 30.0; // seconds
const static char* const ACTION_CONNECTION_FAILED_MSG = "[FF simulation execution] Could not connect to action server.";

// subsribe to the simulation execution action call
const static std::string THIS_SERVICE_NAME = "simulation_execution";

// joint trajectory action to send out
// industrial_robot_simulator in industrial_core packages subsribes to this action and is responsible
// to "move" the robot in rviz for simulation
const static std::string ACTION_SERVER_NAME = "joint_trajectory_action";

choreo_simulation_execution::SimulationExecutionService::SimulationExecutionService(ros::NodeHandle& nh)
    : ac_(ACTION_SERVER_NAME, true)
{
  server_ = nh.advertiseService<SimulationExecutionService, choreo_msgs::TrajectoryExecution::Request,
                                choreo_msgs::TrajectoryExecution::Response>(
      THIS_SERVICE_NAME, &choreo_simulation_execution::SimulationExecutionService::executionCallback, this);

  // Attempt to connect to the motion action service
  if (!ac_.waitForServer(ros::Duration(ACTION_SERVICE_WAIT_TIME)))
  {
    ROS_ERROR_STREAM(ACTION_CONNECTION_FAILED_MSG);
    throw std::runtime_error(ACTION_CONNECTION_FAILED_MSG);
  }
}

bool choreo_simulation_execution::SimulationExecutionService::executionCallback(
    choreo_msgs::TrajectoryExecution::Request& req, choreo_msgs::TrajectoryExecution::Response& res)
{
  // Check preconditions
  if (!ac_.isServerConnected())
  {
    ROS_ERROR_STREAM("[FF Simulation Execution] Simulation Execution Action server is not connected.");
    return false;
  }

  if (req.trajectory.points.empty())
  {
    ROS_WARN_STREAM("[FF Simulation Execution] Trajectory Execution Service recieved an empty trajectory.");
    return true;
  }

  // Populate goal and send
  control_msgs::FollowJointTrajectoryGoal goal;
  goal.trajectory = req.trajectory;
  ac_.sendGoal(goal);

  if (req.wait_for_execution)
  {
    ros::Duration extra_wait =
        goal.trajectory.points.back().time_from_start * ACTION_EXTRA_WAIT_RATIO;
    if (ac_.waitForResult(goal.trajectory.points.back().time_from_start + extra_wait))
    {
      return ac_.getState().state_ == ac_.getState().SUCCEEDED;
    }
    else
    {
      ROS_ERROR_STREAM(__FUNCTION__ << ": Goal failed or did not complete in time.");
      return false;
    }
  }

  return true; // if we don't wait, then always return true immediately
}