//
// Created by yijiangh on 4/13/17.
//

#ifndef FRAMEFABRENDERWIDGET_H
#define FRAMEFABRENDERWIDGET_H

// Qt
#include <QObject>
#include <QWidget>
#include <QSlider>

// ROS
#include <ros/ros.h>
#include <std_msgs/ColorRGBA.h>

// MoveIt
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_msgs/CollisionObject.h>

// framefab
#include <wire_frame.h>

namespace framefab
{

// TODO: this class should be an interface class, share the function of
// scene generation, but keep independent of specific visualizer platform
// (Qt in Rviz for now)

//! @class FrameFabRenderWidget
/*!
 * @brief input model, generate geometry for visualization & computation class
 */
class FrameFabRenderWidget : public QWidget
{
 Q_OBJECT
 public:
  /*!
   * @brief constructor
   * @param[in] ros node_handle
   */
  FrameFabRenderWidget( QWidget* parent = 0 );

  /*!
   * @brief destructor
   */
  ~FrameFabRenderWidget();

  /*!
   * Reads and verifies the ROS parameters.
   * @return true if successful
   */
  bool readParameters();

 public Q_SLOTS:

  /*!
   * @brief (Qt slot function) Read the file name from the dialog and parse results
   */
  void readFile();

  /*!
   * @brief (Qt slot function) publish ros message "draw links"
   */
  void displayPoses();

  /**
   * @brief (Qt slot function) Advances robot one step along current trajectory
   */
  void stepRobot();

  /**
   *
   * @brief sets value of slider
   */
  void setValue(int i);

 private:

  void initCollisionLink(WF_edge* edge, int index, std::vector<moveit_msgs::CollisionObject> * collision_objects);
  void makeCollisionCylinder(WF_edge* edge, int index);

  geometry_msgs::Point transformPoint(point pwf_point);
  geometry_msgs::Pose computeCylinderPose(geometry_msgs::Point start, geometry_msgs::Point center, geometry_msgs::Point end);

 public:
  //! wireframe data structure
  WireFrame* ptr_frame_;

  //! Rendering constants
  float display_point_radius_;
  float pwf_scale_factor_;

  //TODO: this point should be part of user interface in Qt
  geometry_msgs::Point testbed_offset_;

  //TODO: these color should be read from an external ros message file
  std_msgs::ColorRGBA start_color_;
  std_msgs::ColorRGBA end_color_;
  std_msgs::ColorRGBA cylinder_color_;

 private:
  //! Parent pointer for ui updates
  QWidget * parent_;

  //! ROS NodeHandle
  ros::NodeHandle node_handle_;

  //! ROS Rate to refresh Rviz
  ros::Rate*      rate_;

  //! MoveIt! interfaces
  robot_model::RobotModelPtr robot_model_;
  planning_scene_monitor::PlanningSceneMonitor* planning_scene_monitor_;

  //! ROS publisher
  ros::Publisher display_pose_publisher_;

  //! ROS topics
  std::string display_pose_topic_;

};
}/* namespace */

#endif //FRAMEFABRENDERWIDGET_H