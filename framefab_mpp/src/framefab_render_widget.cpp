//
// Created by yijiangh on 4/13/17.
//

// framefab
#include <framefab_render_widget.h>

// util
#include <util/global_functions.h>

// ROS msgs
#include <visualization_msgs/Marker.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseArray.h>

//MoveIt
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

// Qt
#include <QtCore>
#include <QFileDialog>
#include "../include/wire_frame.h"
#include <geometry_msgs/PoseArray.h>
#include "../include/framefab_render_widget.h"
#include "../include/util/global_functions.h"

namespace framefab
{

FrameFabRenderWidget::FrameFabRenderWidget( QWidget* parent )
    : ptr_frame_(NULL)
{
  ROS_INFO("FrameFab Render Widget started.");

  // readParameters
  readParameters();

  // advertise topics - should be done in computation class
  display_marker_publisher_ = node_handle_.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  planning_scene_interface_ = new moveit::planning_interface::PlanningSceneInterface();
  move_group_ = new moveit::planning_interface::MoveGroup("manipulator");

  display_point_radius_ = 0.0025; //TODO: make params
  testbed_offset_.x = 0.1;
  testbed_offset_.y = -0.5;
  testbed_offset_.z = 0.33;
//  // subscribe to RvizPanel's button-emitted topics
//  display_pose_subsriber_ = node_handle_.subscribe(
//      display_pose_topic_, 1,
//      &FrameFabRenderWidget::displayPoseCallBack,
//      this);

//  read_file_subsriber_  = node_handle_.subscribe(
//      read_file_topic_, 1,
//      &FrameFabRenderWidget::readFileCallBack,
//      this);
}

FrameFabRenderWidget::~FrameFabRenderWidget()
{
  framefab::safeDelete(ptr_frame_);
}

bool FrameFabRenderWidget::readParameters()
{
  // FrameFab Parameters
  node_handle_.param("display_pose_topic", display_pose_topic_, std::string("/framelinks"));
  node_handle_.param("read_file_topic", read_file_topic_, std::string("/readfile"));

  return true;
}


moveit_msgs::CollisionObject FrameFabRenderWidget::makeCollisionCylinder(geometry_msgs::Point start, geometry_msgs::Point end,std::string id) {
  moveit_msgs::CollisionObject collision_object;

  collision_object.id = id;
  collision_object.header.frame_id = move_group_->getPlanningFrame();
  collision_object.operation = moveit_msgs::CollisionObject::ADD;
  shape_msgs::SolidPrimitive link_cylinder;
  link_cylinder.type = shape_msgs::SolidPrimitive::CYLINDER;
  link_cylinder.dimensions.resize(2);

  shape_msgs::SolidPrimitive vertex_start;
  shape_msgs::SolidPrimitive vertex_end;
  vertex_start.type = shape_msgs::SolidPrimitive::SPHERE;
  vertex_end.type = shape_msgs::SolidPrimitive::SPHERE;
  vertex_start.dimensions.resize(1);
  vertex_end.dimensions.resize(1);

  vertex_start.dimensions[0] = display_point_radius_;
  vertex_end.dimensions[0] = display_point_radius_;


  Eigen::Vector3d eStart, eEnd;
  tf::pointMsgToEigen(start, eStart);
  tf::pointMsgToEigen(end, eEnd);
  double height = (eStart-eEnd).lpNorm<2>();
  link_cylinder.dimensions[0] = height;
  link_cylinder.dimensions[1] = display_point_radius_;

  Eigen::Vector3d axis = eEnd - eStart;
  axis.normalize();
  Eigen::Vector3d zVec(0.0,0.0,1.0);
  Eigen::Vector3d xVec = axis.cross(zVec);
  xVec.normalize();
  double theta = axis.dot(zVec);
  double angle = -1.0 * acos(theta);

  tf::Vector3 tf_right_axis_vector;
  tf::vectorEigenToTF(xVec, tf_right_axis_vector);
  Eigen::Quaterniond q;
  // Create quaternion
  tf::Quaternion tf_q(tf_right_axis_vector, angle);

  // Convert back to Eigen
  tf::quaternionTFToEigen(tf_q, q);
  q.normalize();
  Eigen::Affine3d pose;
  pose = q * Eigen::AngleAxisd(-0.5*M_PI, Eigen::Vector3d::UnitY());
  Eigen::Vector3d origin;
  tf::pointMsgToEigen(testbed_offset_, origin);
  pose.translation() = origin + eStart;
  geometry_msgs::Pose start_pose;
  geometry_msgs::Pose link_pose;
  geometry_msgs::Pose end_pose;
  start_pose.position = transformPoint(start);
  end_pose.position = transformPoint(end);
  tf::poseEigenToMsg(pose , link_pose);
  collision_object.primitive_poses.push_back(start_pose);
  collision_object.primitives.push_back(vertex_start);
  //collision_object.primitive_poses.push_back(link_pose);
  //collision_object.primitives.push_back(link_cylinder);
  collision_object.primitive_poses.push_back(end_pose);
  collision_object.primitives.push_back(vertex_end);
  // Drawing frame w/ no collision
  //nodes.points.push_back(start);
  //nodes.points.push_back(end);
  //marker_pub.publish(nodes);
  //marker_pub.publish(link_list);



  return collision_object;
}

geometry_msgs::Point FrameFabRenderWidget::transformPoint(geometry_msgs::Point pwf_point) {
  pwf_point.x += testbed_offset_.x;
  pwf_point.y += testbed_offset_.y;
  pwf_point.z += testbed_offset_.z; //TODO likely need to find lowest link and offset all by some amount
  return pwf_point;
}

void FrameFabRenderWidget::displayPoses()
{
  if (NULL == ptr_frame_ ||  0 == ptr_frame_->SizeOfVertList())
  {
    ROS_INFO("Input frame empty, no links to draw.");
    return;
  }

  const std::vector<WF_vert*> wf_verts = *(ptr_frame_->GetVertList());
  for (size_t i = 0; i < wf_verts.size(); i++) {

  }

//
//  std::pair<int,int> edge = edges_[0];
//  geometry_msgs::Pose pose_a;
//  geometry_msgs::Pose pose_b;
//  pose_a.position = scale(nodes_[edge.first], 0.001);
//  pose_b.position = scale(nodes_[edge.second], 0.001);
//
//  pose_msgs.poses.push_back( pose_a);
//  pose_msgs.poses.push_back( pose_b);
//  std::cout << "Publishing points: " << nodes_[edge.first] << " " <<  nodes_[edge.second] << std::endl;
//  edges_.pop_front();
//
//  display_marker_publisher_.publish(msg);

  ROS_INFO("MSG: link pose visualize has been published");
}

void FrameFabRenderWidget::readFile()
{
  QString filename = QFileDialog::getOpenFileName(
      this,
      tr("Open File"),
      "$HOME/Documents",
      tr("pwf Files (*.pwf)"));

  if(filename.isEmpty())
  {
    ROS_ERROR("Read Model Failed!");
    return;
  }

  // compatible with paths in Chinese
  QTextCodec *code = QTextCodec::codecForName("gd18030");
  QTextCodec::setCodecForLocale(code);
  QByteArray byfilename = filename.toLocal8Bit();

  delete ptr_frame_;
  ptr_frame_ = new WireFrame();

  if (filename.contains(".obj") || filename.contains(".OBJ"))
  {
    ptr_frame_->LoadFromOBJ(byfilename.data());
  }
  else
  {
    ptr_frame_->LoadFromPWF(byfilename.data());
  }

  //todo: emit input model info
  QString parse_msg = "Nodes: "       + QString::number(ptr_frame_->SizeOfVertList()) + "\n"
                    + " Links: "    + QString::number(ptr_frame_->SizeOfEdgeList()) + "\n"
                    + " Pillars: "  + QString::number(ptr_frame_->SizeOfPillar()) + "\n"
                    + " Ceilings: " + QString::number(ptr_frame_->SizeOfCeiling());

  ROS_INFO_STREAM("MSG:" << parse_msg.toStdString());
  ROS_INFO("model loaded successfully");
}

//geometry_msgs::Point FrameFabRenderWidget::scale(geometry_msgs::Point p, float sf)
//{
//  geometry_msgs::Point result;
//  result.x = p.x * sf;
//  result.y = p.y * sf;
//  result.z = p.z * sf;
//  return result;
//}

} /* namespace */