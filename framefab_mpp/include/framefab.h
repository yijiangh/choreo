/*
 * FrameFab.h
 * 
 * Created on:  April 7, 2017
 * Author:      Yijiang Huang, Thomas Cook
 * Institute:   MIT, Digital Structure Group, Building Tech
*/

#ifndef FRAMEFAB_H
#define FRAMEFAB_H

// ROS
#include <ros/ros.h>

namespace framefab {

/*! @class FrameFab
 *  @brief The framefab main container class.
 *
 *  The framefab main container class. Coordinates the ROS interface
 *  and the data handling between the other classes.
*/
class FrameFab {

  /* public functions */
 public:

  /*!
   *	@brief Constructor.
   * 	@param[in] ROS nodeHandle for parameters request
   */
  FrameFab(ros::NodeHandle &node_handle);

  /*!
   *	@brief Destructor.
   */
  ~FrameFab();

  /*
   * function for development, experiment goes here.
   */
  void debug();

  /* private functions */
 private:
  /*!
   * Reads and verifies the ROS parameters.
   * @return true if successful
   */
  bool readParameters();

  /*!
  * Performs the initialization procedure.
  * @return true if successful.
  */
  bool initialize() {}

  /* private data */
 private:

  //! ROS nodehandle.
  ros::NodeHandle &node_handle_;

  //! ROS subscribers.

  //! ROS topics for subscriptions.

};

} /* namespace */

#endif // FRAMEFAB_H