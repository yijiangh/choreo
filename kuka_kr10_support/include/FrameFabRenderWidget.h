//
// Created by yijiangh on 4/13/17.
//

#ifndef FRAMEFABRENDERWIDGET_H
#define FRAMEFABRENDERWIDGET_H

// Qt
#include <QObject>

// ROS
#include <ros/ros.h>

namespace framefab {

    //! @class FrameFabRenderWidget
    /*!
     * @brief framefab UI interaction coordinator
     *
     */
    class FrameFabRenderWidget
    {
    Q_OBJECT

    public:
        /*!
         * @brief constructor
         * @param[in] ros node_handle
         */
        FrameFabRenderWidget(ros::NodeHandle &node_handle);

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
        void drawLink();

    private:
        geometry_msgs::Point scale(geometry_msgs::Point p, float sf);


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
    };
}/* namespace */

#endif //FRAMEFABRENDERWIDGET_H
