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

  ROS_INFO_STREAM("decartes: input world_frame: " << world_frame << ", base_frame: " << getBaseFrame());

  auto tip_frames = getTipFrames();
  for(auto tcp_f : tip_frames)
  {
    ROS_INFO_STREAM("decartes: tip_frame: " << tcp_f);
  }

  ROS_INFO_STREAM("group name: " << getGroupName());
  auto link_names = getLinkNames();
  for(auto str : link_names)
  {
    ROS_INFO_STREAM("descartes: link name - " << str);
  }

  auto joint_names = getJointNames();
  for(auto str : joint_names)
  {
    ROS_INFO_STREAM("descartes: joint name - " << str);
  }

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
