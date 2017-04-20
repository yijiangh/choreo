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

// MoveIt
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

// framefab
#include <wire_frame.h>

namespace framefab
{

//! @class FrameFabRenderWidget
/*!
 * @brief framefab UI interaction coordinator
 *
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
   * @brief Read the file name from the dialog and parse results
   */
  void readFile();

  /*!
   * @brief publish ros message "draw links"
   */
  void displayPoses();
  /**
   * @brief Advances robot one step along current trajectory
   */
  void stepRobot();

 private:
  void makeCollisionCylinder(WF_edge* edge, std::string id);
  geometry_msgs::Point transformPoint(point pwf_point);
  geometry_msgs::Pose computeCylinderPose(geometry_msgs::Point start, geometry_msgs::Point end);
  moveit_msgs::CollisionObject initCollisionLink(WF_edge* edge, std::string id);

 public:
  //! wireframe data structure
  WireFrame* ptr_frame_;

  //! MoveIt interfaces
  moveit::planning_interface::PlanningSceneInterface * planning_scene_interface_;
  moveit::planning_interface::MoveGroup * move_group_;

  //! Rendering constants
  float display_point_radius_;
  float pwf_scale_factor_;
  geometry_msgs::Point testbed_offset_;


 private:
  //! ROS NodeHandle
  ros::NodeHandle node_handle_;

  //! ROS subscriber
  ros::Subscriber read_file_subsriber_;

  //! ROS publisher
  ros::Publisher display_marker_publisher_;
  ros::Publisher display_pose_publisher_;
  //! ROS topics
  std::string display_pose_topic_;
  std::string read_file_topic_;

};
}/* namespace */

#endif //FRAMEFABRENDERWIDGET_H
