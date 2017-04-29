//
// Created by yijiangh on 4/13/17.
//

// ROS msgs
#include <visualization_msgs/Marker.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseArray.h>

//MoveIt
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

// Qt
#include <QtCore>
#include <QFileDialog>

// framefab
#include <wire_frame/wire_frame_line_graph.h>
#include <framefab_rviz_panel.h>
#include <util/global_functions.h>

namespace framefab
{

FrameFabRenderWidget::FrameFabRenderWidget( QWidget* parent )
    : ptr_frame_(NULL),
      ptr_framefab_(NULL),
      parent_(parent)
{
  ROS_INFO("FrameFab Render Widget started.");

  // readParameters
  readParameters();

  // TODO: does this rate belongs to a node?
  rate_ = new ros::Rate(10.0);
  //rate->sleep();

  // advertise topics - should be done in computation class
  // display_marker_publisher_ = node_handle_.advertise<visualization_msgs::Marker>(
  // "visualization_marker", 1 , true);
  display_pose_publisher_ = node_handle_.advertise<moveit_msgs::DisplayTrajectory>(
      "/move_group/display_planned_path", 1 ,true);

  // TODO: move this motion topic into external ros parameters
  // should figure out a way to extract rviz nodehandle
  // the member variable now seems not to be the rviz one.
  // check:
  //  ROS_INFO(ros::this_node::getName().c_str());
  //  std::string a = node_handle_.getNamespace();
  //  ROS_INFO(a.c_str());
  std::ostringstream motion_topic;
  motion_topic << "/rviz_ubuntu_24663_1410616146488788559" << "/monitored_planning_scene";

  planning_scene_monitor_ = new planning_scene_monitor::PlanningSceneMonitor("robot_description");

  //planning_scene_monitor_->startSceneMonitor(motion_topic.str());
  //params
  display_point_radius_ = 0.0025; //TODO: make params
  testbed_offset_.x = 0.1;
  testbed_offset_.y = -0.5;
  testbed_offset_.z = 0.33;
  pwf_scale_factor_ = 0.001; // mm to m
  start_color_.r = 0;
  start_color_.g = 255;
  start_color_.b = 0;
  start_color_.a = 1;
  end_color_.r = 255;
  end_color_.g = 0;
  end_color_.b = 0;
  end_color_.a = 1;
  cylinder_color_.r = 122;
  cylinder_color_.g = 122;
  cylinder_color_.b = 122;
  cylinder_color_.a = 0.25;

  planning_scene_monitor_->startPublishingPlanningScene(
      planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE, motion_topic.str());
}

FrameFabRenderWidget::~FrameFabRenderWidget()
{
  framefab::safeDelete(ptr_frame_);
  framefab::safeDelete(ptr_framefab_);
}

bool FrameFabRenderWidget::readParameters()
{
  // FrameFab Parameters
  node_handle_.param("display_pose_topic", display_pose_topic_, std::string("/framelinks"));
  return true;
}

void FrameFabRenderWidget::initCollisionLink(
    wire_frame::WF_edge* edge, int index, std::vector<moveit_msgs::CollisionObject>* collision_objects)
{
  //TODO consider preallocating/preprocessing

  std::string frame_id = planning_scene_monitor_->getPlanningScene()->getPlanningFrame();

  /*
   * Collision_cylinder: a CollisionObject container,
   * containing two endnodes and linking truss member
   */
  moveit_msgs::CollisionObject collision_cylinder;
  std::ostringstream cyl_id;
  cyl_id << "cylinder" << index;
  collision_cylinder.id = cyl_id.str();
  collision_cylinder.header.frame_id = frame_id;//TODO investigate whether worth to make class variable for efficiency
  collision_cylinder.operation = moveit_msgs::CollisionObject::ADD;

  // cylinder shape that model the truss member
  shape_msgs::SolidPrimitive link_cylinder;
  link_cylinder.type = shape_msgs::SolidPrimitive::CYLINDER;
  link_cylinder.dimensions.resize(2);
  link_cylinder.dimensions[0] = pwf_scale_factor_ * edge->Length(); //TODO check
  link_cylinder.dimensions[1] = display_point_radius_;

  /*
   * collision_start_node: spheres that model two end nodes
   */
  // TODO: the node id should made unique, for future DELETE operation
  // consider using wireframe node id convention
  moveit_msgs::CollisionObject collision_start_node;
  moveit_msgs::CollisionObject collision_end_node;
  std::ostringstream start_id;
  std::ostringstream end_id;
  start_id << "start_node" << index;
  end_id << "end_node" << index;
  collision_start_node.id = start_id.str();
  collision_end_node.id = end_id.str();
  collision_start_node.header.frame_id = frame_id;//TODO investigate whether worth to make class variable for efficiency
  collision_start_node.operation = moveit_msgs::CollisionObject::ADD;
  collision_end_node.header.frame_id = frame_id; //TODO investigate whether worth to make class variable for efficiency
  collision_end_node.operation = moveit_msgs::CollisionObject::ADD;

  // sphere shape that model the two end nodes
  shape_msgs::SolidPrimitive vertex_start;
  shape_msgs::SolidPrimitive vertex_end;
  vertex_start.type = shape_msgs::SolidPrimitive::SPHERE;
  vertex_end.type = shape_msgs::SolidPrimitive::SPHERE;
  vertex_start.dimensions.resize(1);
  vertex_end.dimensions.resize(1);

  //set the resolution for spheres and cylinder
  vertex_start.dimensions[0] = display_point_radius_;
  vertex_end.dimensions[0] = display_point_radius_;

  //set poses
  geometry_msgs::Point start = transformPoint(edge->pvert_->Position());
  geometry_msgs::Point end = transformPoint(edge->ppair_->pvert_->Position());
  geometry_msgs::Point center = transformPoint(edge->CenterPos());
  geometry_msgs::Pose start_pose;
  geometry_msgs::Pose end_pose;
  start_pose.position = start;
  end_pose.position = end;

  collision_start_node.primitives.push_back(vertex_start);
  collision_start_node.primitive_poses.push_back(start_pose);
  collision_start_node.primitives.resize(1);
  collision_start_node.primitive_poses.resize(1);

  collision_cylinder.primitives.push_back(link_cylinder);
  collision_cylinder.primitive_poses.push_back(computeCylinderPose(start, center ,end));
  collision_cylinder.primitives.resize(1);
  collision_cylinder.primitive_poses.resize(1);

  collision_end_node.primitives.push_back(vertex_end);
  collision_end_node.primitive_poses.push_back(end_pose);
  collision_end_node.primitives.resize(1);
  collision_end_node.primitive_poses.resize(1);

  collision_objects->push_back(collision_start_node);
  collision_objects->push_back(collision_cylinder);
  collision_objects->push_back(collision_end_node);
  collision_objects->resize(3);

  planning_scene::PlanningScenePtr current = planning_scene_monitor_->getPlanningScene();
  current->setObjectColor(cyl_id.str(), cylinder_color_);
  current->setObjectColor(start_id.str(),start_color_);
  current->setObjectColor(end_id.str(), end_color_);
}

/**
 *
 * @param edge
 * @param id
 */
void FrameFabRenderWidget::makeCollisionCylinder(wire_frame::WF_edge* edge , int index)
{
  std::vector<moveit_msgs::CollisionObject> links;
  initCollisionLink(edge, index, &links);
  moveit_msgs::PlanningScene scene;
  planning_scene_monitor_->getPlanningScene()->getPlanningSceneMsg(scene);

  // TODO: make a class to store the collision objects
  // this is the main reason why this is slow
  // resulting in collision objects queue like:
  // 1-12-123-1234... (massive unnecessary creation of collision objects)
  // from m (number of edges) -> m * (m + 1) / 2
  for (int i=0; i < links.size(); i++)
  {
    scene.world.collision_objects.push_back(links[i]);
  }
  scene.is_diff = 1;
  planning_scene_monitor_->newPlanningSceneMessage(scene);
  //rate_->sleep();

}

/**
 * Computes quaternion transformation for cylinder with base at start oriented toward end with length = dist(start, end)
 * where dist(start, end) is Euclidean distance between points start,end in R^3
 * @param start geometry_msgs::Point starting point in R^3
 * @param end geometry_msgs::Point endpoint in R^3
 * @return orientation and position of cylinder collision object wrt testbed offset as origin
 */
geometry_msgs::Pose FrameFabRenderWidget::computeCylinderPose(
    geometry_msgs::Point start, geometry_msgs::Point center, geometry_msgs::Point end)
{
  Eigen::Vector3d e_start, e_end;
  tf::pointMsgToEigen(start, e_start);
  tf::pointMsgToEigen(end, e_end);

  //rotation
  Eigen::Vector3d axis = e_end - e_start;
  axis.normalize();
  Eigen::Vector3d z_vec(0.0,0.0,1.0);
  const Eigen::Vector3d &x_vec = axis.cross(z_vec);
  double theta = axis.dot(z_vec);
  double angle = -1.0 * acos(theta);
  tf::Vector3 xVec;
  tf::vectorEigenToTF(x_vec, xVec);
  // Create quaternion
  tf::Quaternion tf_q(xVec , angle);
  tf_q.normalize();

  //back to ros coords
  geometry_msgs::Pose link_pose;
  tf::quaternionTFToMsg(tf_q, link_pose.orientation);
  link_pose.position = center;

  return link_pose;

}

/**
 * Takes R^3 point from pwf file and converts to units for rviz
 * @param pwf_point
 * @return
 */
geometry_msgs::Point FrameFabRenderWidget::transformPoint(trimesh::point  pwf_point)
{
  geometry_msgs::Point point;
  point.x += testbed_offset_.x + (pwf_scale_factor_ * pwf_point.x());
  point.y += testbed_offset_.y + (pwf_scale_factor_ * pwf_point.y());
  point.z += testbed_offset_.z + (pwf_scale_factor_ * pwf_point.z());

  //TODO likely need to find lowest link and offset all by some amount
  return point;
}

void FrameFabRenderWidget::displayPoses()
{
  using wire_frame::WF_edge;

  if (NULL == ptr_frame_ ||  0 == ptr_frame_->SizeOfVertList())
  {
    ROS_INFO("Input frame empty, no links to draw.");
    return;
  }

  const std::vector<WF_edge*> wf_edges = *(ptr_frame_->GetEdgeList());
  for (size_t i = 0; i < wf_edges.size(); i++)
  {
    makeCollisionCylinder(wf_edges[i], i);
  }
  rate_->sleep();

  ROS_INFO("MSG: link pose visualize has been published");
}

void FrameFabRenderWidget::advanceRobot()
{
  // init main computation class - FrameFab here
  ROS_INFO("Renderwidget: advance robot called");

  // init framefab
  safeDelete(ptr_framefab_);
  ptr_framefab_ = new FrameFab(node_handle_);

  ptr_framefab_->debug();
}

void FrameFabRenderWidget::setValue(int i) {

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

  framefab::safeDelete(ptr_frame_);
  ptr_frame_ = new wire_frame::WireFrameLineGraph();

  if (filename.contains(".obj") || filename.contains(".OBJ"))
  {
    ptr_frame_->LoadFromOBJ(byfilename.data());
  }
  else
  {
    ptr_frame_->LoadFromPWF(byfilename.data());
  }

  //todo: emit input model info
  QString parse_msg = "Nodes: "     + QString::number(ptr_frame_->SizeOfVertList()) + "\n"
                    + " Links: "    + QString::number(ptr_frame_->SizeOfEdgeList()) + "\n"
                    + " Pillars: "  + QString::number(ptr_frame_->SizeOfPillar()) + "\n"
                    + " Ceilings: " + QString::number(ptr_frame_->SizeOfCeiling());
  ((FrameFabRvizPanel*)parent_)->console(parse_msg);
  ROS_INFO_STREAM("MSG:" << parse_msg.toStdString());
  ROS_INFO("model loaded successfully");
}

} /* namespace framefab */