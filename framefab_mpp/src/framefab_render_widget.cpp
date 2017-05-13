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
#include <QString>

// framefab
#include <framefab_rviz_panel.h>
#include <framefab_render_widget.h>

namespace framefab
{

FrameFabRenderWidget::FrameFabRenderWidget( QWidget* parent )
    : parent_(parent),
      unit_conversion_scale_factor_(1),
      ref_pt_x_(300), ref_pt_y_(0), ref_pt_z_(0)
{
  ROS_INFO("[framefab_mpp] FrameFabPlanner Render Widget started.");

  node_handle_ = ros::NodeHandle("framefab_render_widget");

  // readParameters
  readParameters();

  rate_ = new ros::Rate(10.0);

  // init publisher, name in global space
  display_pose_publisher_ = node_handle_.advertise<moveit_msgs::DisplayTrajectory>(
      "/move_group/display_planned_path", 1 ,true);

  // init collision objects
  ROS_INFO("[framefab_mpp] init ptr_wf_collision");
  ptr_wire_frame_collision_objects_ = boost::make_shared<wire_frame::WireFrameCollisionObjects>();

  // TODO: move this motion topic into external ros parameters
  // should figure out a way to extract rviz nodehandle
  // the member variable now seems not to be the rviz one.

  std::ostringstream motion_topic;
  motion_topic << "/move_group" << "/monitored_planning_scene";

  ptr_planning_scene_monitor_ =
      boost::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");

  ptr_planning_scene_monitor_->startPublishingPlanningScene(
      planning_scene_monitor::PlanningSceneMonitor::UPDATE_GEOMETRY, "/move_group/monitored_planning_scene");

  ptr_planning_scene_interface_ = boost::make_shared<moveit::planning_interface::PlanningSceneInterface>();
}

FrameFabRenderWidget::~FrameFabRenderWidget()
{
}

bool FrameFabRenderWidget::readParameters()
{
  // render widget topic name parameters
  node_handle_.param("display_pose_topic", display_pose_topic_, std::string("/framelinks"));

  // rviz render parameters
  node_handle_.param("/framefab_mpp/wire_frame_collision_start_vertex_color_r", start_color_.r, float(0.0));
  node_handle_.param("/framefab_mpp/wire_frame_collision_start_vertex_color_g", start_color_.g, float(0.0));
  node_handle_.param("/framefab_mpp/wire_frame_collision_start_vertex_color_b", start_color_.b, float(0.0));
  node_handle_.param("/framefab_mpp/wire_frame_collision_start_vertex_color_a", start_color_.a, float(0.0));

  node_handle_.param("/framefab_mpp/wire_frame_collision_end_vertex_color_r", end_color_.r, float(0.0));
  node_handle_.param("/framefab_mpp/wire_frame_collision_end_vertex_color_g", end_color_.g, float(0.0));
  node_handle_.param("/framefab_mpp/wire_frame_collision_end_vertex_color_b", end_color_.b, float(0.0));
  node_handle_.param("/framefab_mpp/wire_frame_collision_end_vertex_color_a", end_color_.a, float(0.0));

  node_handle_.param("/framefab_mpp/wire_frame_collision_cylinder_color_r", cylinder_color_.r, float(0.0));
  node_handle_.param("/framefab_mpp/wire_frame_collision_cylinder_color_g", cylinder_color_.g, float(0.0));
  node_handle_.param("/framefab_mpp/wire_frame_collision_cylinder_color_b", cylinder_color_.b, float(0.0));
  node_handle_.param("/framefab_mpp/wire_frame_collision_cylinder_color_a", cylinder_color_.a, float(0.0));

  node_handle_.param("/framefab_mpp/wire_frame_collision_cylinder_radius", collision_cylinder_radius_, float(0.0001));
  return true;
}

void FrameFabRenderWidget::displayPoses()
{
}

void FrameFabRenderWidget::advanceRobot()
{
  // init main computation class - FrameFabPlanner here
  ROS_INFO("[framefab_mpp] Renderwidget: advance robot called");

  // implement using framefab_planner
//  ptr_framefab_planner_ = boost::make_shared<FrameFabPlanner>();
//
//  ptr_framefab_planner_->debug();

  ptr_move_group_ = boost::make_shared<move_group_interface::MoveGroup>("manipulator");

  moveit::planning_interface::MoveGroup::Plan my_plan;
  std::vector<geometry_msgs::Pose> waypoints;

  geometry_msgs::Point start = ptr_move_group_->getCurrentPose().pose.position;
  geometry_msgs::Pose start_pose, next_pose;

  next_pose = ptr_wire_frame_collision_objects_->getCollisionObject(0)->start_vertex_collision.primitive_poses[0];

  waypoints.push_back(next_pose);
  moveit_msgs::RobotTrajectory trajectory;

  double fraction = ptr_move_group_->computeCartesianPath(waypoints, 0.001, 0.00, trajectory);

  if (fraction > 0.1)
  {
    my_plan.trajectory_ = trajectory;
    ROS_INFO("Fraction %.2f",fraction);
    ptr_move_group_->asyncExecute(my_plan);
    rate_->sleep();
  }
  else
  {
    ROS_INFO("Fraction to small");
    return;
  }
}

void FrameFabRenderWidget::setValue(int i)
{

}

void FrameFabRenderWidget::setScaleFactor(QString unit_info)
{
  Q_EMIT(sendLogInfo(QString("-----------MODEL UNIT-----------")));
  Q_EMIT(sendLogInfo(unit_info));

  if(QString("millimeter") == unit_info)
  {
    node_handle_.param("/framefab_mpp/unit_conversion_millimeter_to_meter", unit_conversion_scale_factor_, float(1));
  }

  if(QString("centimeter") == unit_info)
  {
    node_handle_.param("/framefab_mpp/unit_conversion_centimeter_to_meter", unit_conversion_scale_factor_, float(1));
  }

  if(QString("inch") == unit_info)
  {
    node_handle_.param("/framefab_mpp/unit_conversion_inch_to_meter", unit_conversion_scale_factor_, float(1));
  }

  if(QString("foot") == unit_info)
  {
    node_handle_.param("/framefab_mpp/unit_conversion_foot_to_meter", unit_conversion_scale_factor_, float(1));
  }

  Q_EMIT(sendLogInfo(QString("Convert to meter - scale factor %1").arg(unit_conversion_scale_factor_)));

  // if collision objects exists, rebuild
  if(0 != ptr_wire_frame_collision_objects_->sizeOfCollisionObjectsList())
  {
    ROS_INFO("rebuilding model");
    Q_EMIT(sendLogInfo(QString("Model rebuilt for update on unit.")));
    this->constructCollisionObjects();
  }
}

void FrameFabRenderWidget::setRefPoint(double ref_pt_x, double ref_pt_y, double ref_pt_z)
{
  Q_EMIT(sendLogInfo(QString("-----------REF POINT-----------")));
  Q_EMIT(sendLogInfo(QString("(%1, %2, %3)").arg(ref_pt_x).arg(ref_pt_y).arg(ref_pt_z)));

  ref_pt_x_ = ref_pt_x;
  ref_pt_y_ = ref_pt_y;
  ref_pt_z_ = ref_pt_z;

  // if collision objects exists, rebuild
  if(0 != ptr_wire_frame_collision_objects_->sizeOfCollisionObjectsList())
  {
    ROS_INFO("rebuilding model");
    Q_EMIT(sendLogInfo(QString("Model rebuilt for update on Ref Point.")));
    this->constructCollisionObjects();
  }
}

void FrameFabRenderWidget::constructCollisionObjects()
{
  //--------------- load wireframe collision objects start -----------------------
  ptr_wire_frame_collision_objects_->constructCollisionObjects(
      ptr_planning_scene_monitor_->getPlanningScene()->getPlanningFrame(),
      unit_conversion_scale_factor_, collision_cylinder_radius_,
      ref_pt_x_, ref_pt_y_, ref_pt_z_);

  wire_frame::MoveitLinearMemberCollisionObjectsListPtr ptr_collision_objects
      = ptr_wire_frame_collision_objects_->getCollisionObjectsList();

  std::vector<std::string> objects_id = ptr_planning_scene_interface_->getKnownObjectNames();
  ptr_planning_scene_interface_->removeCollisionObjects(objects_id);

  std::vector<moveit_msgs::CollisionObject> collision_objects;

  // TODO: planning scene not cleaned up in second time input case, ref issue #21
  for (int i=0; i < ptr_collision_objects->size(); i++)
  {
    collision_objects.push_back((*ptr_collision_objects)[i]->edge_cylinder_collision);
    //scene.world.collision_objects.push_back((*ptr_collision_objects)[i]->edge_cylinder_collision);
    //scene.world.collision_objects.push_back((*ptr_collision_objects)[i]->start_vertex_collision);
    //scene.world.collision_objects.push_back((*ptr_collision_objects)[i]->end_vertex_collision);


    //moveit_msgs::ObjectColor cylinder_moveit_color;
    //cylinder_moveit_color.id = (*ptr_collision_objects)[i]->start_vertex_collision.id.c_str();
    //cylinder_moveit_color.color = cylinder_color_;

    //scene.object_colors.push_back(cylinder_moveit_color);
    //ptr_current_scene->setObjectColor((*ptr_collision_objects)[i]->start_vertex_collision.id.c_str(), start_color_);
    //ptr_current_scene->setObjectColor((*ptr_collision_objects)[i]->end_vertex_collision.id.c_str(), end_color_);
  }
  //--------------- load wireframe collision objects end -----------------------

  ptr_planning_scene_interface_->addCollisionObjects(collision_objects);

  rate_->sleep();
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
  else
  {
    ROS_INFO("Open Model: success.");
  }

  // compatible with paths in Chinese
  QTextCodec *code = QTextCodec::codecForName("gd18030");
  QTextCodec::setCodecForLocale(code);
  QByteArray byfilename = filename.toLocal8Bit();

  ptr_wire_frame_collision_objects_.reset();
  ptr_wire_frame_collision_objects_ = boost::make_shared<wire_frame::WireFrameCollisionObjects>();

  //--------------- load wireframe linegraph end -----------------------
  if (filename.contains(".obj") || filename.contains(".OBJ"))
  {
    ptr_wire_frame_collision_objects_->LoadFromOBJ(byfilename.data());
  }
  else
  {
    ptr_wire_frame_collision_objects_->LoadFromPWF(byfilename.data());
  }

  if (0 == ptr_wire_frame_collision_objects_->SizeOfVertList())
  {
    Q_EMIT(sendLogInfo(QString("Input frame empty, no links to draw.")));
    return;
  }

  ROS_INFO("LineGraph Built: success.");
  //--------------- load wireframe linegraph end -----------------------

  this->constructCollisionObjects();

  QString parse_msg =
      "Nodes: "     + QString::number(ptr_wire_frame_collision_objects_->SizeOfVertList()) + "\n"
          + "Links: "    + QString::number(ptr_wire_frame_collision_objects_->SizeOfEdgeList()) + "\n"
          + "Pillars: "  + QString::number(ptr_wire_frame_collision_objects_->SizeOfPillar()) + "\n"
          + "Ceilings: " + QString::number(ptr_wire_frame_collision_objects_->SizeOfCeiling());

  Q_EMIT(sendLogInfo(QString("-----------MODEL INPUT-----------")));
  Q_EMIT(sendLogInfo(parse_msg));
  Q_EMIT(sendLogInfo(QString("factor scale: %1").arg(unit_conversion_scale_factor_)));

  ROS_INFO("model loaded successfully");
}

} /* namespace framefab */