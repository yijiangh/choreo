//
// Created by yijiangh on 4/13/17.
//

#ifndef PLANNING_WIDGET_H
#define PLANNING_WIDGET_H

// Qt
#include <QObject>
#include <QWidget>
#include <QString>

// ROS
#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
//#include <framefab_msgs/PathPlanningParameters.h>
#include <framefab_msgs/ProcessPlanningAction.h>

const static std::string PROCESS_PLANNING_ACTION_SERVER_NAME = "process_planning_as";

namespace framefab_rviz_plugin
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

 Q_SIGNALS:
 void sendLogInfo(QString);

 public Q_SLOTS:

  /*!
   * @brief (Qt slot function) Read the file name from the dialog and parse results
   */
  void readFile();

  /**
   * @brief (Qt slot function) Advances robot one step along current trajectory
   *
   * Current a descartes planning testbed.
   */
  void advanceRobot();
  void testDescartes();

  void setScaleFactor(QString unit_info);
  void setRefPoint(double ref_pt_x, double ref_pt_y, double ref_pt_z);
  void setCartPlanningOffsetPoint(double move_x, double move_y, double move_z);

  void constructCollisionObjects();

 private:
  double scaleData(double l) { return l * (double)unit_conversion_scale_factor_; }

 private:
  //! Model constants
  float collision_cylinder_radius_;
  float unit_conversion_scale_factor_;

  double ref_pt_x_;
  double ref_pt_y_;
  double ref_pt_z_;

  double move_offset_x_;
  double move_offset_y_;
  double move_offset_z_;

  //! Model rendering constants
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
  ros::ServiceClient adv_robot_srv_client_;
  ros::ServiceClient test_descartes_srv_client_;

  moveit::planning_interface::PlanningSceneInterfacePtr ptr_planning_scene_interface_;
  move_group_interface::MoveGroupPtr ptr_move_group_;

  //! ROS topics
  std::string display_pose_topic_;

  //! wireframe data structure
  wire_frame::WireFrameCollisionObjectsPtr ptr_wire_frame_collision_objects_;
};
}/* namespace */

#endif //PLANNING_WIDGET_H