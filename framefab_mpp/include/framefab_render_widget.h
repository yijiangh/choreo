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

// framefab - wireframe
#include <wire_frame/Vec.h>
#include <wire_frame/wire_frame_line_graph.h>
#include <wire_frame/wire_frame_collision_objects.h>

#include <framefab.h>

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

  // TODO: this should not be Poses (just collision object visualization)
  // TODO: the orientation for robot should leave to be determined later
  /*!
   * @brief (Qt slot function) publish ros message "draw links"
   */
  void displayPoses();

  /**
   * @brief (Qt slot function) Advances robot one step along current trajectory
   *
   * Current a descartes planning testbed.
   */
  void advanceRobot();

  /**
   *
   * @brief sets value of slider
   */
  void setValue(int i);

 private:

  void initCollisionLink(
      wire_frame::WF_edge* edge, int index, std::vector<moveit_msgs::CollisionObject> * collision_objects);
  void makeCollisionCylinder(
      wire_frame::WF_edge* edge, int index);

  geometry_msgs::Point transformPoint(trimesh::point pwf_point);

/*!
 * Computes quaternion transformation for cylinder with base at start oriented toward end with length = dist(start, end)
 * where dist(start, end) is Euclidean distance between points start,end in R^3
 * @param start geometry_msgs::Point starting point in R^3
 * @param end geometry_msgs::Point endpoint in R^3
 * @return orientation and position of cylinder collision object wrt testbed offset as origin
 */
  geometry_msgs::Pose computeCylinderPose(
      geometry_msgs::Point start, geometry_msgs::Point center, geometry_msgs::Point end);

 private:
  //! Rendering constants
  float display_point_radius_;
  float pwf_scale_factor_;

  //TODO: this point should be part of user interface in Qt
  geometry_msgs::Point testbed_offset_;

  //TODO: these color should be read from an external ros message file
  std_msgs::ColorRGBA start_color_;
  std_msgs::ColorRGBA end_color_;
  std_msgs::ColorRGBA cylinder_color_;

  //! Parent pointer for ui updates
  QWidget* parent_;

  //! ROS NodeHandle
  ros::NodeHandle node_handle_;

  //! ROS Rate to refresh Rviz
  ros::Rate*      rate_;

  //! MoveIt! interfaces
  robot_model::RobotModelPtr                      ptr_robot_model_;
  planning_scene_monitor::PlanningSceneMonitorPtr ptr_planning_scene_monitor_;

  //! ROS publisher
  ros::Publisher display_pose_publisher_;

  //! ROS topics
  std::string display_pose_topic_;

  // TODO: use smart_ptr
  //! FrameFab computation class
  FrameFab* ptr_framefab_;

  //! wireframe data structure
  wire_frame::WireFrameCollisionObjectsPtr ptr_wire_frame_collision_objects_;
};
}/* namespace */

#endif //FRAMEFABRENDERWIDGET_H