/*
 * FrameFab.h
 * 
 * Created on: April 7, 2017
 * Author:		 Yijiang Huang, Thomas Cook
 * Institute:  MIT, Digital Structure Group, Building Tech 
*/

#pragma once



namespace framefab {

/*! @class FrameFab
 *  @brief The framefab main container class.
 * 
 *  The framefab main container class. Coordinates the ROS interface
 *  and the data handling between the other classes.
*/
class FrameFab
{
public:

	/*!
	 *	@brief Constructor.
	 * 	@param[in] nodeHandle the ROS node handle
	 */
	FrameFab(ros::NodeHandle& nodeHandle);

	/*!
	 *	@brief Destructor.
	 */
	~FrameFab();

}

} /* namespace */
