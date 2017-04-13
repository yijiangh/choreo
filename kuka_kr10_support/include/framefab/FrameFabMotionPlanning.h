/*
 * FrameFabMotionPlanning.h
 *
 * Created on:  April 7, 2017
 * Author:      Yijiang Huang, Thomas Cook
 * Institute:   MIT, Digital Structure Group, Building Tech
*/

#ifndef FRAMEFAB_MOTIONPLANNING_H
#define FRAMEFAB_MOTIONPLANNING_H

// ROS
#include <ros/ros.h>

namespace framefab {

    /*! @class FrameFab
     *  @brief The framefab main container class.
     *
     *  The framefab main container class. Coordinates the ROS interface
     *  and the data handling between the other classes.
    */
    class FrameFabMotionPlanning {

        /* public functions */
    public:
        /*!
         *	@brief Constructor.
         * 	@param[in] nodeHandle the ROS node handle
         */
        FrameFabMotionPlanning(ros::NodeHandle nodeHandle);

        /*!
         *	@brief Constructor.
         *
         */
        ~FrameFabMotionPlanning() {}

        /*!
         * Reads and verifies the ROS parameters.
         * @return true if successful.
         */
        bool readParameters();

        /* private data */
    private:

        //! Ros nodehandle
        ros::NodeHandle *ptr_node_handle_;
    };

} /* namespace */

#endif