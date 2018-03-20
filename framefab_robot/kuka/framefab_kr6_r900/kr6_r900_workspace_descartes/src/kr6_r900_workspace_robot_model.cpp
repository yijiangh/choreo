#include <kr6_r900_workspace_descartes/kr6_r900_workspace_robot_model.h>
#include <eigen_conversions/eigen_kdl.h>
#include <pluginlib/class_list_macros.h>

static const std::string DEFAULT_BASE_LINK_NAME = "robot_base_link";
static const std::string DEFAULT_TOOL_LINK_NAME = "robot_tool0";
static const double JOINT_LIMIT_TOLERANCE = .0000001f;

using namespace descartes_moveit;

namespace kr6_r900_workspace_descartes
{
Kr6R900WorkspaceRobotModel::Kr6R900WorkspaceRobotModel()
    : world_to_base_(Eigen::Affine3d::Identity()), tool_to_tip_(Eigen::Affine3d::Identity())
{
}

bool Kr6R900WorkspaceRobotModel::initialize(const std::string& robot_description,
                                      const std::string& group_name, const std::string& world_frame,
                                      const std::string& tcp_frame)
{
	  // look up the IKFast base and tool frame
  ros::NodeHandle nh;
  std::string ikfast_base_frame, ikfast_tool_frame;
  nh.param<std::string>("ikfast_base_frame", ikfast_base_frame, DEFAULT_BASE_LINK_NAME);
  nh.param<std::string>("ikfast_tool_frame", ikfast_tool_frame, DEFAULT_TOOL_LINK_NAME);

  MoveitStateAdapter::initialize(robot_description, group_name, world_frame, tcp_frame);
  kr6_r900_workspace_ikfast_manipulator_plugin::IKFastKinematicsPlugin::initialize(
      robot_description, group_name, ikfast_base_frame, ikfast_tool_frame, 0.001);

  if (!robot_state_->knowsFrameTransform(ikfast_base_frame))
  {
    logError("IkFastMoveitStateAdapter: Cannot find transformation to frame '%s' in group '%s'.",
            ikfast_base_frame.c_str(), group_name.c_str());
    return false;
  }

  if (!robot_state_->knowsFrameTransform(ikfast_tool_frame))
  {
    logError("IkFastMoveitStateAdapter: Cannot find transformation to frame '%s' in group '%s'.",
            ikfast_tool_frame.c_str(), group_name.c_str());
    return false;
  }

  // calculate frames
  tool_to_tip_ = descartes_core::Frame(robot_state_->getFrameTransform(tcp_frame).inverse() *
      robot_state_->getFrameTransform(ikfast_tool_frame));

  world_to_base_ = descartes_core::Frame(world_to_root_.frame * robot_state_->getFrameTransform(ikfast_base_frame));

  logInform("customized Descartes ikfast plugin: initialized with IKFast tool frame '%s' and base frame '%s'.",
            ikfast_tool_frame.c_str(), ikfast_base_frame.c_str());

  return true;
}

bool Kr6R900WorkspaceRobotModel::getAllIK(const Eigen::Affine3d& pose,
                                    std::vector<std::vector<double> >& joint_poses) const
{
	using namespace kr6_r900_workspace_ikfast_manipulator_plugin;

  std::vector<double> vfree(this->free_params_.size(), 0.0);
  KDL::Frame frame;
  Eigen::Affine3d tool_pose = world_to_base_.frame_inv * pose * tool_to_tip_.frame;
  tf::transformEigenToKDL(tool_pose, frame);

  ikfast::IkSolutionList<IkReal> solutions;

  int numsol = IKFastKinematicsPlugin::solve(frame, vfree, solutions);

  joint_poses.clear();

  if (numsol)
  {
    for (int s = 0; s < numsol; ++s)
    {
      std::vector<double> sol;
      getSolution(solutions, s, sol);

      if (isValid(sol))
        joint_poses.push_back(sol);

      // So, IKFast returns the unique configurations of the robot (e.g. elbow up, wrist down)
      // and the solutions have joint values between -pi and +pi. If the robot can rotate more
      // than this, then we need to check to see if we have extra solutions that have the same
      // configuration but a different joint position. In our case, joint 6 has this kind of
      // extra motion, so here we check for valid solutions 360 degrees from each solution.
      sol[5] += 2 * M_PI;
      if (isValid(sol))
        joint_poses.push_back(sol);

      sol[5] -= 2 * 2 * M_PI;
      if (isValid(sol))
        joint_poses.push_back(sol);

    }
  }

  return !joint_poses.empty();
}

}

PLUGINLIB_EXPORT_CLASS(kr6_r900_workspace_descartes::Kr6R900WorkspaceRobotModel, descartes_core::RobotModel)
