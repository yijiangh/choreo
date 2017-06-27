//
// Created by yijiangh on 6/26/17.
//
#include <framefab_path_post_processor/process_path.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

double dist(const Eigen::Vector3d& from, const Eigen::Vector3d& to)
{
  return (from - to).norm();
}

geometry_msgs::Pose framefab_util::UnitProcessPath::computeCylinderPose() const
{
  // rotation
  Eigen::Vector3d axis = end_pt - st_pt_;
  axis.normalize();
  Eigen::Vector3d z_vec(0.0,0.0,1.0);
  const Eigen::Vector3d& x_vec = axis.cross(z_vec);
  double theta = axis.dot(z_vec);
  double angle = -1.0 * acos(theta);

  // convert eigen vertor to tf::Vector3
  tf::Vector3 x_vec_tf;
  tf::vectorEigenToTF(x_vec, x_vec_tf);

  // Create quaternion
  tf::Quaternion tf_q(x_vec_tf , angle);
  tf_q.normalize();

  //back to ros coords
  geometry_msgs::Pose cylinder_pose;
  tf::quaternionTFToMsg(tf_q, cylinder_pose.orientation);
  cylinder_pose.position = (end_pt + st_pt_) * 0.5;

  return cylinder_pose;
}

void framefab_utils::UnitProcessPath::createCollisionObject()
{
  std::string cylinder_id = "element_" + std::to_string(i);

  // TODO: make frame_id as input parameter
  collision_cylinder_.id = cylinder_id;
  collision_cylinder_.header.frame_id = "arm_base_link";
  collision_cylinder_.operation = moveit_msgs::CollisionObject::ADD;

  // TODO: turn cylinder radiuus as input parameter
  shape_msgs::SolidPrimitive cylinder_solid;
  collision_cylinder_.type = shape_msgs::SolidPrimitive::CYLINDER;
  collision_cylinder_.dimensions.resize(2);
  collision_cylinder_.dimensions[0] = dist(st_pt_, end_pt_);
  collision_cylinder_.dimensions[1] = 0.0015;
  collision_cylinder_.primitives.resize(1);
  collision_cylinder_.primitives.push_back(cylinder_solid);

  geometry_msgs::Point cylinder_center_point = transformPoint(GetEdge(i)->CenterPos());
  collision_cylinder_.primitive_poses.resize(1);
  collision_cylinder_.primitive_poses.push_back(computeCylinderPose());
}

framefab_msgs::ElementCandidatePoses framefab_utils::UnitProcessPath::asElementCandidatePoses() const
{


}
