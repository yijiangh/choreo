//
// Created by yijiangh on 4/13/17.
//

#ifndef FRAMEFABRENDERWIDGET_H
#define FRAMEFABRENDERWIDGET_H

// Qt
#include <QObject>

// Rviz
#include <rviz/panel.h>

// ROS
#include <ros/ros.h>

// framefab
#include <WireFrame.h>

namespace framefab
{

//! @class FrameFabRenderWidget
/*!
 * @brief framefab UI interaction coordinator
 *
 */
class FrameFabRenderWidget : public rviz::Panel
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

 private:
  //geometry_msgs::Point scale(geometry_msgs::Point p, float sf);

 public:
  //! wireframe data structure
  WireFrame* ptr_frame_;

 private:
  //! ROS NodeHandle
  ros::NodeHandle node_handle_;

  //! ROS subscriber
  ros::Subscriber display_pose_subsriber_;
  ros::Subscriber read_file_subsriber_;

  //! ROS publisher
  ros::Publisher display_marker_publisher_;

  //! ROS topics
  std::string display_pose_topic_;
  std::string read_file_topic_;

  // todo: to be replaced, only for testing
  //! display links id
  int display_id_;
};
}/* namespace */

#endif //FRAMEFABRENDERWIDGET_H
