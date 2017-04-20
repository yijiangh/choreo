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
  display_marker_publisher_ = node_handle_.advertise<visualization_msgs::Marker>("visualization_marker", 1 , true);
  display_pose_publisher_ = node_handle_.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1 ,true);
  planning_scene_interface_ = new moveit::planning_interface::PlanningSceneInterface();
  move_group_ = new moveit::planning_interface::MoveGroup("manipulator");

  //params
  display_point_radius_ = 0.0025; //TODO: make params
  testbed_offset_.x = 0.1;
  testbed_offset_.y = -0.5;
  testbed_offset_.z = 0.33;
  pwf_scale_factor_ = 0.001; // mm to m


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

/**
 *
 * @param edge
 * @param id
 * @return
 */
moveit_msgs::CollisionObject FrameFabRenderWidget::initCollisionLink(WF_edge* edge, std::string id) {
  //TODO consider preallocating/preprocessing
  //define object
  moveit_msgs::CollisionObject collision_object;
  collision_object.id = id;
  collision_object.header.frame_id = move_group_->getPlanningFrame(); //TODO investigate whether worth to make class variable for efficiency
  collision_object.operation = moveit_msgs::CollisionObject::ADD;
  collision_object.primitives.resize(3);
  collision_object.primitive_poses.resize(3);

  //set up cylinder that models the link between two nodes
  shape_msgs::SolidPrimitive link_cylinder;
  link_cylinder.type = shape_msgs::SolidPrimitive::CYLINDER;
  link_cylinder.dimensions.resize(2);

  //set up the spheres that model the nodes
  shape_msgs::SolidPrimitive vertex_start;
  shape_msgs::SolidPrimitive vertex_end;
  vertex_start.type = shape_msgs::SolidPrimitive::SPHERE;
  vertex_end.type = shape_msgs::SolidPrimitive::SPHERE;
  vertex_start.dimensions.resize(1);
  vertex_end.dimensions.resize(1);

  //set the resolution for spheres and cylinder
  vertex_start.dimensions[0] = display_point_radius_;
  vertex_end.dimensions[0] = display_point_radius_;
  link_cylinder.dimensions[0] = display_point_radius_;
  link_cylinder.dimensions[1] = edge->Length();

  //set poses
  geometry_msgs::Point start = transformPoint(edge->pvert_->Position());
  geometry_msgs::Point end = transformPoint(edge->ppair_->pvert_->Position());
  geometry_msgs::Pose start_pose;
  geometry_msgs::Pose end_pose;
  start_pose.position = start;
  end_pose.position = end;

  collision_object.primitive_poses[0] = start_pose;
  collision_object.primitives[0] = vertex_start;

  collision_object.primitives[1] = link_cylinder;
  collision_object.primitive_poses[1] = computeCylinderPose(start, end);

  collision_object.primitive_poses[2] = end_pose;
  collision_object.primitives[2] = vertex_end;
  return collision_object;
}

/**
 *
 * @param edge
 * @param id
 */
void FrameFabRenderWidget::makeCollisionCylinder(WF_edge* edge , std::string id)
{

  moveit_msgs::CollisionObject collision_object = initCollisionLink(edge,id);
  std::vector<moveit_msgs::CollisionObject> links;
  links.push_back(collision_object);

  planning_scene_interface_->addCollisionObjects(links);
  sleep(10);


}

/**
 * Computes quaternion transformation for cylinder with base at start oriented toward end with length = dist(start, end)
 * where dist(start, end) is Euclidean distance between points start,end in R^3
 * @param start geometry_msgs::Point starting point in R^3
 * @param end geometry_msgs::Point endpoint in R^3
 * @return orientation and position of cylinder collision object wrt testbed offset as origin
 */
geometry_msgs::Pose FrameFabRenderWidget::computeCylinderPose(geometry_msgs::Point start, geometry_msgs::Point end) {
  //transform upright cylinder to cylinder-shaped vector starting at eStart, oriented toward eEnd
  Eigen::Vector3d eStart, eEnd;
  tf::pointMsgToEigen(start, eStart);
  tf::pointMsgToEigen(end, eEnd);

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

  geometry_msgs::Pose link_pose;

  tf::poseEigenToMsg(pose , link_pose);
  return link_pose;

}

/**
 *
 * @param pwf_point
 * @return
 */
geometry_msgs::Point FrameFabRenderWidget::transformPoint(point  pwf_point) {
  geometry_msgs::Point point;
  point.x += testbed_offset_.x + (pwf_scale_factor_ * pwf_point.x());
  point.y += testbed_offset_.y + (pwf_scale_factor_ * pwf_point.y());
  point.z += testbed_offset_.z + (pwf_scale_factor_ * pwf_point.z()); //TODO likely need to find lowest link and offset all by some amount
  return point;
}
/**
 *
 */
void FrameFabRenderWidget::displayPoses()
{
  if (NULL == ptr_frame_ ||  0 == ptr_frame_->SizeOfVertList())
  {
    ROS_INFO("Input frame empty, no links to draw.");
    return;
  }

  const std::vector<WF_edge*> wf_edges = *(ptr_frame_->GetEdgeList());
  for (size_t i = 0; i < wf_edges.size(); i++) {
    makeCollisionCylinder(wf_edges[i], "link");
  }

  ROS_INFO("MSG: link pose visualize has been published");
}

void FrameFabRenderWidget::stepRobot() {

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


} /* namespace framefab */