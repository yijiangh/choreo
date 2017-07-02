//
// Created by yijiangh on 7/1/17.
//

#include <framefab_kr6_r900_descartes/framefab_kr6_r900_robot_model.h>
#include <eigen_conversions/eigen_kdl.h>
#include <pluginlib/class_list_macros.h>

static const std::string DEFAULT_BASE_LINK_NAME = "base_link";
static const std::string DEFAULT_TOOL_LINK_NAME = "tool0";
static const double JOINT_LIMIT_TOLERANCE = .0000001f;

using namespace descartes_moveit;
using namespace framefab_kr6_r900_ikfast_manipulator_plugin;

namespace framefab_kr6_r900_descartes
{
FrameFabKr6R900RobotModel::FrameFabKr6R900RobotModel()
    : world_to_base_(Eigen::Affine3d::Identity()), tool_to_tip_(Eigen::Affine3d::Identity())
{
}

bool FrameFabKr6R900RobotModel::initialize(const std::string& robot_description,
                                      const std::string& group_name, const std::string& world_frame,
                                      const std::string& tcp_frame)
{
  MoveitStateAdapter::initialize(robot_description, group_name, world_frame, tcp_frame);
  framefab_kr6_r900_ikfast_manipulator_plugin::IKFastKinematicsPlugin::initialize(
      robot_description, group_name, DEFAULT_BASE_LINK_NAME, DEFAULT_TOOL_LINK_NAME, 0.001);

  // initialize world transformations
  if (tcp_frame != getTipFrame())
  {
    tool_to_tip_ = descartes_core::Frame(robot_state_->getFrameTransform(tcp_frame).inverse() *
        robot_state_->getFrameTransform(getTipFrame()));
  }

  if (world_frame != getBaseFrame())
  {
    world_to_base_ = descartes_core::Frame(world_to_root_.frame *
        robot_state_->getFrameTransform(getBaseFrame()));
  }

  return true;
}

bool FrameFabKr6R900RobotModel::getAllIK(const Eigen::Affine3d& pose,
                                    std::vector<std::vector<double> >& joint_poses) const
{
  std::vector<double> vfree(free_params_.size(), 0.0);
  KDL::Frame frame;
  Eigen::Affine3d tool_pose = world_to_base_.frame_inv * pose * tool_to_tip_.frame;
  tf::transformEigenToKDL(tool_pose, frame);

  ikfast::IkSolutionList<IkReal> solutions;

  int numsol = solve(frame, vfree, solutions);

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

PLUGINLIB_EXPORT_CLASS(framefab_kr6_r900_descartes::FrameFabKr6R900RobotModel, descartes_core::RobotModel)
