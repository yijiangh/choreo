//
// Created by yijiangh on 6/26/17.
//
#include <ros/ros.h>
#include <ros/console.h>

#include <framefab_task_sequence_processor/unit_process.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

double dist(const Eigen::Vector3d& from, const Eigen::Vector3d& to)
{
  return (from - to).norm();
}

geometry_msgs::Pose framefab_task_sequence_processing_utils::UnitProcess::computeCylinderPose(
    const Eigen::Vector3d& st_pt, const Eigen::Vector3d& end_pt) const
{
  geometry_msgs::Pose cylinder_pose;

  // rotation
  Eigen::Vector3d axis = end_pt - st_pt;
  axis.normalize();
  Eigen::Vector3d z_vec(0.0, 0.0, 1.0);
  const Eigen::Vector3d& x_vec = axis.cross(z_vec);

  tf::Quaternion tf_q;
  if(0 == x_vec.norm())
  {
   // axis = z_vec
    tf_q = tf::Quaternion(0, 0, 0, 1);
  }
  else
  {
    double theta = axis.dot(z_vec);
    double angle = -1.0 * acos(theta);

    // convert eigen vertor to tf::Vector3
    tf::Vector3 x_vec_tf;
    tf::vectorEigenToTF(x_vec, x_vec_tf);

    // Create quaternion
    tf_q = tf::Quaternion(x_vec_tf, angle);
    tf_q.normalize();
  }

  //back to ros coords
  tf::quaternionTFToMsg(tf_q, cylinder_pose.orientation);
  tf::pointEigenToMsg((end_pt + st_pt) * 0.5, cylinder_pose.position);

  return cylinder_pose;
}

moveit_msgs::CollisionObject framefab_task_sequence_processing_utils::UnitProcess::createCollisionObject(
    const int& id, const Eigen::Vector3d& st_pt, const Eigen::Vector3d& end_pt,
    const double& element_diameter, std::string&& shrink_type_suffix) const
{
  moveit_msgs::CollisionObject collision_cylinder;
  std::string cylinder_id = "element_" + std::to_string(id) + "_" + shrink_type_suffix;

  // TODO: make frame_id as input parameter
  collision_cylinder.id = cylinder_id;
  collision_cylinder.header.frame_id = "world_frame";
  collision_cylinder.operation = moveit_msgs::CollisionObject::ADD;

  shape_msgs::SolidPrimitive cylinder_solid;
  cylinder_solid.type = shape_msgs::SolidPrimitive::CYLINDER;
  cylinder_solid.dimensions.resize(2);
  cylinder_solid.dimensions[0] = dist(st_pt, end_pt);
  cylinder_solid.dimensions[1] = element_diameter;
  collision_cylinder.primitives.push_back(cylinder_solid);
  collision_cylinder.primitive_poses.push_back(computeCylinderPose(st_pt, end_pt));

  return collision_cylinder;
}

void framefab_task_sequence_processing_utils::UnitProcess::createShrinkedEndPoint(Eigen::Vector3d& st_pt, Eigen::Vector3d& end_pt,
                    const double& shrink_length)
{
  Eigen::Vector3d translation_vec = end_pt - st_pt;
  translation_vec.normalize();

  st_pt = st_pt + shrink_length * translation_vec;
  end_pt = end_pt - shrink_length * translation_vec;
}

framefab_msgs::ElementCandidatePoses framefab_task_sequence_processing_utils::UnitProcess::asElementCandidatePoses()
{
  framefab_msgs::ElementCandidatePoses msg;

  // element index
  msg.element_id = id_;
  msg.wireframe_id = wireframe_id_;

  // process element's type
  if("support" == type_)
  {
    msg.type = framefab_msgs::ElementCandidatePoses::SUPPORT;
  }
  if("create" == type_)
  {
    msg.type = framefab_msgs::ElementCandidatePoses::CREATE;
  }
  if("connect" == type_)
  {
    msg.type = framefab_msgs::ElementCandidatePoses::CONNECT;
  }

  // add start end point data
  tf::pointEigenToMsg(st_pt_, msg.start_pt);
  tf::pointEigenToMsg(end_pt_, msg.end_pt);

  msg.element_diameter = element_diameter_;

  Eigen::Vector3d shrinked_st_pt = st_pt_;
  Eigen::Vector3d shrinked_end_pt = end_pt_;

  createShrinkedEndPoint(shrinked_st_pt, shrinked_end_pt, shrink_length_);

  tf::pointEigenToMsg(shrinked_st_pt, msg.shrinked_start_pt);
  tf::pointEigenToMsg(shrinked_end_pt, msg.shrinked_end_pt);

  msg.both_side_shrinked_collision_object = createCollisionObject(id_, shrinked_st_pt, shrinked_end_pt,
                                                                  element_diameter_, "shrink_both_ends");

  msg.st_shrinked_collision_object = createCollisionObject(id_, shrinked_st_pt, end_pt_,
                                                           element_diameter_, "shrink_start");

  msg.end_shrinked_collision_object = createCollisionObject(id_, st_pt_, shrinked_end_pt,
                                                            element_diameter_, "shrink_end");


//  // for safety, we inflate the full collision geometry
//  Eigen::Vector3d extended_st_pt = st_pt_;
//  Eigen::Vector3d extended_end_pt = end_pt_;
//  createShrinkedEndPoint(extended_st_pt, extended_st_pt, - shrink_length_);

  msg.full_collision_object = createCollisionObject(id_, st_pt_, end_pt_,
                                                    element_diameter_, "full");


  // add feasible EEF orientations
  for(int i=0; i < feasible_orients_.size(); i++)
  {
    geometry_msgs::Vector3 vec_msg;
    tf::vectorEigenToMsg(feasible_orients_[i], vec_msg);

    msg.feasible_orients.push_back(vec_msg);
  }

  return msg;
}
