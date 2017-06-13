//
// Created by yijiangh on 4/13/17.
//

#include <map>
#include <boost/shared_ptr.hpp>

// ROS msgs
#include <visualization_msgs/Marker.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseArray.h>
#include <framefab_msgs/AdvanceRobot.h>
#include <framefab_msgs/TestDescartes.h>

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
      ref_pt_x_(0), ref_pt_y_(0), ref_pt_z_(0),
      move_offset_x_(0), move_offset_y_(0), move_offset_z_(0)
{
  ROS_INFO("[ff_render_widget] FrameFabPlanner Render Widget started.");

  node_handle_ = ros::NodeHandle("framefab_render_widget");
  rate_ = new ros::Rate(10.0);

  // readParameters
  readParameters();

  // init collision objects
  ROS_INFO("[ff_render_widget] init ptr_wf_collision");
  ptr_wire_frame_collision_objects_ = boost::make_shared<wire_frame::WireFrameCollisionObjects>();

  ptr_planning_scene_interface_ = boost::make_shared<moveit::planning_interface::PlanningSceneInterface>();

  ptr_move_group_ = boost::make_shared<move_group_interface::MoveGroup>("manipulator");

  adv_robot_srv_client_ = node_handle_.serviceClient<framefab_msgs::AdvanceRobot>(
      "/framefab_mpp_node/advance_robot");

  test_descartes_srv_client_ = node_handle_.serviceClient<framefab_msgs::TestDescartes>(
      "/framefab_mpp_node/test_descartes");
}

FrameFabRenderWidget::~FrameFabRenderWidget()
{
}

void FrameFabRenderWidget::setScaleFactor(QString unit_info)
{
//  Q_EMIT(sendLogInfo(QString("-----------MODEL UNIT-----------")));
//  Q_EMIT(sendLogInfo(unit_info));

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

//  Q_EMIT(sendLogInfo(QString("Convert to meter - scale factor %1").arg(unit_conversion_scale_factor_)));

  // if collision objects exists, rebuild
  if(0 != ptr_wire_frame_collision_objects_->sizeOfCollisionObjectsList())
  {
    ROS_INFO_NAMED("framefab_mpp", "[FF_RenderWidget] rebuilding model");
    Q_EMIT(sendLogInfo(QString("Model rebuilt for update on unit.")));
    this->constructCollisionObjects();
  }
}



void FrameFabRenderWidget::setCartPlanningOffsetPoint(double move_x, double move_y, double move_z)
{
  Q_EMIT(sendLogInfo(QString("-----------ROBOT RELATIVE MOVE-----------")));

  move_offset_x_ = move_x;
  move_offset_y_ = move_y;
  move_offset_z_ = move_z;
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
    ROS_ERROR_NAMED("framefab_mpp", "[FF_RenderWidget] Read Model Failed!");
    return;
  }
  else
  {
    ROS_INFO_NAMED("framefab_mpp", "[FF_RenderWidget] Open Model: success.");
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

  ROS_INFO_NAMED("framefab_mpp", "[FF_RenderWidget] LineGraph Built: success.");
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

  ROS_INFO_NAMED("framefab_mpp", "[FF_RenderWidget] model loaded successfully");
}

} /* namespace framefab */