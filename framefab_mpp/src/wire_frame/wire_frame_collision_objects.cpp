//
// Created by yijiangh on 4/29/17.
//

#include <boost/make_shared.hpp>

#include <ros/ros.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

#include <wire_frame/wire_frame_collision_objects.h>


namespace framefab
{
namespace wire_frame
{

WireFrameCollisionObjects::WireFrameCollisionObjects() : WireFrameLineGraph()
{
  ptr_linear_member_collision_objects_list_ = boost::make_shared<MoveitLinearMemberCollisionObjectsList>();

  unit_scale_factor_ = 1;
  ref_point_transf_vec_ = trimesh::vec3(0,0,0);
}

void WireFrameCollisionObjects::constructCollisionObjects(
    const std::string frame_id, const float unit_scale_factor,
    const double collision_cylinder_radius,
    const double ref_pt_x, const double ref_pt_y, const double ref_pt_z)
{
  int m = this->SizeOfEdgeList();

  // reload scale factor and ref_point
  unit_scale_factor_ = unit_scale_factor;
  ref_point_transf_vec_.x() = (ref_pt_x - this->GetBaseCenterPos().x()) * unit_scale_factor_;
  ref_point_transf_vec_.y() = (ref_pt_y - this->GetBaseCenterPos().y()) * unit_scale_factor_;
  ref_point_transf_vec_.z() = (ref_pt_z - this->GetBaseCenterPos().z()) * unit_scale_factor_;

  // clear and rebuild collision obj list
  ptr_linear_member_collision_objects_list_->clear();

  for(int i=0; i < m; i++)
  {
    MoveitLinearMemberCollisionObjectsPtr ptr_member_collision_obj =
        boost::make_shared<MoveitLinearMemberCollisionObjects>();

    // get WF_edge
    WF_edge *ptr_wf_e = GetEdge(i);
    int start_vertex_id_u = ptr_wf_e->ppair_->pvert_->ID();
    int end_vertex_id_v = ptr_wf_e->pvert_->ID();

    // start node setup
    std::string start_vertex_id = "start_vertex-" + std::to_string(start_vertex_id_u);
    ptr_member_collision_obj->start_vertex_collision.id = start_vertex_id;
    ptr_member_collision_obj->start_vertex_collision.header.frame_id = frame_id;
    ptr_member_collision_obj->start_vertex_collision.operation
        = moveit_msgs::CollisionObject::ADD;

    shape_msgs::SolidPrimitive vertex_start_solid;
    vertex_start_solid.type = shape_msgs::SolidPrimitive::SPHERE;
    vertex_start_solid.dimensions.resize(1);
    vertex_start_solid.dimensions[0] = collision_cylinder_radius;

    geometry_msgs::Pose vertex_start_pose;
    geometry_msgs::Point vertex_start_point = transformPoint(
        GetPosition(start_vertex_id_u));
    vertex_start_pose.position = transformPoint(
        GetPosition(start_vertex_id_u));

    ptr_member_collision_obj->start_vertex_collision.primitives.push_back(vertex_start_solid);
    ptr_member_collision_obj->start_vertex_collision.primitive_poses.push_back(vertex_start_pose);
    ptr_member_collision_obj->start_vertex_collision.primitives.resize(1);
    ptr_member_collision_obj->start_vertex_collision.primitive_poses.resize(1);

    // end node setup
    std::string end_vertex_id = "end_vertex-" + std::to_string(end_vertex_id_v);
    ptr_member_collision_obj->end_vertex_collision.id = end_vertex_id;
    ptr_member_collision_obj->end_vertex_collision.header.frame_id = frame_id;
    ptr_member_collision_obj->end_vertex_collision.operation
        = moveit_msgs::CollisionObject::ADD;

    shape_msgs::SolidPrimitive vertex_end_solid;
    vertex_end_solid.type = shape_msgs::SolidPrimitive::SPHERE;
    vertex_end_solid.dimensions.resize(1);
    vertex_end_solid.dimensions[0] = collision_cylinder_radius;

    geometry_msgs::Pose vertex_end_pose;
    geometry_msgs::Point vertex_end_point = transformPoint(
        GetPosition(end_vertex_id_v));
    vertex_end_pose.position = transformPoint(
        GetPosition(end_vertex_id_v));

    ptr_member_collision_obj->end_vertex_collision.primitives.push_back(vertex_end_solid);
    ptr_member_collision_obj->end_vertex_collision.primitive_poses.push_back(vertex_end_pose);
    ptr_member_collision_obj->end_vertex_collision.primitives.resize(1);
    ptr_member_collision_obj->end_vertex_collision.primitive_poses.resize(1);

    // cylinder setup
    std::string cylinder_id = "cylinder-" + std::to_string(i);

    ptr_member_collision_obj->edge_cylinder_collision.id = cylinder_id;
    ptr_member_collision_obj->edge_cylinder_collision.header.frame_id = frame_id;
    ptr_member_collision_obj->edge_cylinder_collision.operation
        = moveit_msgs::CollisionObject::ADD;

    shape_msgs::SolidPrimitive cylinder_solid;
    cylinder_solid.type = shape_msgs::SolidPrimitive::CYLINDER;
    cylinder_solid.dimensions.resize(2);
    cylinder_solid.dimensions[0] = unit_scale_factor * ptr_wf_e->Length();
    cylinder_solid.dimensions[1] = collision_cylinder_radius;

    geometry_msgs::Point cylinder_center_point = transformPoint(
        GetEdge(i)->CenterPos());
    ptr_member_collision_obj->edge_cylinder_collision.primitives.push_back(cylinder_solid);
    ptr_member_collision_obj->edge_cylinder_collision.primitive_poses.push_back(
        computeCylinderPose(vertex_start_point, cylinder_center_point, vertex_end_point));
    ptr_member_collision_obj->edge_cylinder_collision.primitives.resize(1);
    ptr_member_collision_obj->edge_cylinder_collision.primitive_poses.resize(1);

    ptr_linear_member_collision_objects_list_->push_back(ptr_member_collision_obj);
  }
}

  geometry_msgs::Point WireFrameCollisionObjects::transformPoint(const trimesh::point pwf_point)
  {
    geometry_msgs::Point g_point;
    g_point.x = pwf_point.x() * unit_scale_factor_ + ref_point_transf_vec_.x();
    g_point.y = pwf_point.y() * unit_scale_factor_ + ref_point_transf_vec_.y();
    g_point.z = pwf_point.z() * unit_scale_factor_ + ref_point_transf_vec_.z();

    return g_point;
  }

  geometry_msgs::Pose WireFrameCollisionObjects::computeCylinderPose(
      const geometry_msgs::Point st_point,
      const geometry_msgs::Point center_point,
      const geometry_msgs::Point end_point)
  {
    Eigen::Vector3d e_start, e_end;
    tf::pointMsgToEigen(st_point, e_start);
    tf::pointMsgToEigen(end_point, e_end);

    // rotation
    Eigen::Vector3d axis = e_end - e_start;
    axis.normalize();
    Eigen::Vector3d z_vec(0.0,0.0,1.0);
    const Eigen::Vector3d &x_vec = axis.cross(z_vec);
    double theta = axis.dot(z_vec);
    double angle = -1.0 * acos(theta);

    // convert eigen vertor to tf::Vector3
    tf::Vector3 xVec;
    tf::vectorEigenToTF(x_vec, xVec);

    // Create quaternion
    tf::Quaternion tf_q(xVec , angle);
    tf_q.normalize();

    //back to ros coords
    geometry_msgs::Pose cylinder_pose;
    tf::quaternionTFToMsg(tf_q, cylinder_pose.orientation);
    cylinder_pose.position = center_point;

    return cylinder_pose;
  }

}// namespace wire_frame
}// namespace framefab
