//
// Created by yijiangh on 4/13/17.
//

#include <boost/shared_ptr.hpp>

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
#include <framefab_rviz_panel.h>
#include <util/global_functions.h>

namespace framefab
{

FrameFabRenderWidget::FrameFabRenderWidget( QWidget* parent )
    : ptr_framefab_(NULL),
      parent_(parent)
{
  ROS_INFO("FrameFab Render Widget started.");

  // readParameters
  readParameters();

  // TODO: does this rate belongs to a node?
  rate_ = new ros::Rate(10.0);
  //rate->sleep();

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

  ptr_planning_scene_monitor_ =
      boost::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");

  ptr_planning_scene_monitor_->startPublishingPlanningScene(
      planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE, motion_topic.str());

  //planning_scene_monitor_->startSceneMonitor(motion_topic.str());
  //params
  display_point_radius_ = 0.0025; //TODO: make params
  testbed_offset_.x = 0.1;
  testbed_offset_.y = -0.5;
  testbed_offset_.z = 0.33;
  pwf_scale_factor_ = 0.001; // mm to m
}

FrameFabRenderWidget::~FrameFabRenderWidget()
{
  framefab::safeDelete(ptr_framefab_);
}

bool FrameFabRenderWidget::readParameters()
{
  // render widget topic name parameters
  node_handle_.param("display_pose_topic", display_pose_topic_, std::string("/framelinks"));

  // rviz render parameters
  node_handle_.param("wire_frame_collision_start_vertex_color_r", start_color_.r, float(0.0));
  node_handle_.param("wire_frame_collision_start_vertex_color_g", start_color_.g, float(0.0));
  node_handle_.param("wire_frame_collision_start_vertex_color_b", start_color_.b, float(0.0));
  node_handle_.param("wire_frame_collision_start_vertex_color_a", start_color_.a, float(0.0));

  node_handle_.param("wire_frame_collision_end_vertex_color_r", end_color_.r, float(0.0));
  node_handle_.param("wire_frame_collision_end_vertex_color_g", end_color_.g, float(0.0));
  node_handle_.param("wire_frame_collision_end_vertex_color_b", end_color_.b, float(0.0));
  node_handle_.param("wire_frame_collision_end_vertex_color_a", end_color_.a, float(0.0));

  node_handle_.param("wire_frame_collision_cylinder_color_r", cylinder_color_.r, float(0.0));
  node_handle_.param("wire_frame_collision_cylinder_color_g", cylinder_color_.g, float(0.0));
  node_handle_.param("wire_frame_collision_cylinder_color_b", cylinder_color_.b, float(0.0));
  node_handle_.param("wire_frame_collision_cylinder_color_a", cylinder_color_.a, float(0.0));

  return true;
}

void FrameFabRenderWidget::displayPoses()
{
  using wire_frame::WF_edge;

  if (nullptr == ptr_wire_frame_collision_objects_  ||  0 == ptr_wire_frame_collision_objects_->SizeOfVertList())
  {
    ROS_INFO("Input frame empty, no links to draw.");
    return;
  }

  trimesh::point offset_point(testbed_offset_.x, testbed_offset_.y, testbed_offset_.z);

  ptr_wire_frame_collision_objects_->constructCollisionObjects(
      ptr_planning_scene_monitor_, pwf_scale_factor_, display_point_radius_, offset_point);

  wire_frame::MoveitLinearMemberCollisionObjectsListPtr ptr_collision_objects
      = ptr_wire_frame_collision_objects_->getCollisionObjectsList();

  moveit_msgs::PlanningScene scene;
  ptr_planning_scene_monitor_->getPlanningScene()->getPlanningSceneMsg(scene);
  planning_scene::PlanningScenePtr ptr_current_scene = ptr_planning_scene_monitor_->getPlanningScene();

  for (int i=0; i < ptr_collision_objects->size(); i++)
  {
    scene.world.collision_objects.push_back((*ptr_collision_objects)[i]->start_vertex_collision);
    scene.world.collision_objects.push_back((*ptr_collision_objects)[i]->edge_cylinder_collision);
    scene.world.collision_objects.push_back((*ptr_collision_objects)[i]->end_vertex_collision);


    ptr_current_scene->setObjectColor((*ptr_collision_objects)[i]->edge_cylinder_collision.id.c_str(), cylinder_color_);
    ptr_current_scene->setObjectColor((*ptr_collision_objects)[i]->start_vertex_collision.id.c_str(), start_color_);
    ptr_current_scene->setObjectColor((*ptr_collision_objects)[i]->end_vertex_collision.id.c_str(), end_color_);
  }

  scene.is_diff = 1;
  ptr_planning_scene_monitor_->newPlanningSceneMessage(scene);

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

void FrameFabRenderWidget::setValue(int i)
{

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

  ptr_wire_frame_collision_objects_.reset();

  ptr_wire_frame_collision_objects_ = boost::make_shared<wire_frame::WireFrameCollisionObjects>();

  if (filename.contains(".obj") || filename.contains(".OBJ"))
  {
    ptr_wire_frame_collision_objects_->LoadFromOBJ(byfilename.data());
  }
  else
  {
    ptr_wire_frame_collision_objects_->LoadFromPWF(byfilename.data());
  }

  //todo: emit input model info
  QString parse_msg = "Nodes: "     + QString::number(ptr_wire_frame_collision_objects_->SizeOfVertList()) + "\n"
      + " Links: "    + QString::number(ptr_wire_frame_collision_objects_->SizeOfEdgeList()) + "\n"
      + " Pillars: "  + QString::number(ptr_wire_frame_collision_objects_->SizeOfPillar()) + "\n"
      + " Ceilings: " + QString::number(ptr_wire_frame_collision_objects_->SizeOfCeiling());
  ((FrameFabRvizPanel*)parent_)->console(parse_msg);
  ROS_INFO_STREAM("MSG:" << parse_msg.toStdString());
  ROS_INFO("model loaded successfully");
}

} /* namespace framefab */