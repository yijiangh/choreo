/*
 * FrameFab.h
 * 
 * Created on: April 7, 2017
 * Author:		 Yijiang Huang, Thomas Cook
 * Institute:  MIT, Digital Structure Group, Building Tech 
*/

#ifndef FRAMEFAB_H
#define FRAMEFAB_H

namespace framefab {

/*! @class FrameFab
 *  @brief The framefab main container class.
 * 
 *  The framefab main container class. Coordinates the ROS interface
 *  and the data handling between the other classes.
*/
class FrameFab
{
    /* public functions */
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

    /* private functions */
private:

    /*!
     * Reads and verifies the ROS parameters.
     * @return true if successful
     */
     bool readParameters();

    /* private data */
private:

    //! ROS nodehandle.
    ros::NodeHandle& nodeHandle_;

    //! RSO subscribers.

};

} /* namespace */

#endif // FRAMEFAB_H