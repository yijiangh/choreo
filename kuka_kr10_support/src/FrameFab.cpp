/*
 * FrameFab.h
 *
 * Created on: April 7, 2017
 * Author:		 Yijiang Huang, Thomas Cook
 * Institute:  MIT, Digital Structure Group, Building Tech
*/

#include <framefab/FrameFab.h>

namespace framefab{
    Framefab::Framefab(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle)
    {
        ROS_INFO("FrameFab node started.");

        // message subscriber declaration


    }
}
