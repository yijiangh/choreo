#ifndef FRAMEFAB_PANEL_H
#define FRAMEFAB_PANEL_H

// Qt
#include <QLineEdit>
#include <QPushButton>
#include <QTextEdit>

// Rviz
#include <rviz/panel.h>

// ROS
#include <ros/ros.h>

namespace framefab {

    //! @class FrameFabPanel
    /*!
     * @brief framefab UI panel for Rviz
     */
    class FrameFabPanel: public rviz::Panel
{
    Q_OBJECT

    public:
        FrameFabPanel( QWidget* parent = 0 );
        ~FrameFabPanel() {}

        // Now we declare overrides of rviz::Panel functions for saving and
        // loading data from the config file.  Here the data is the
        // topic name.
        virtual void load( const rviz::Config& config );
        virtual void save( rviz::Config config ) const;

    public Q_SLOTS:

        /*!
         * @brief Read the file name from the dialog and parse results
         */
        void readFile();

        /*!
         * @brief publish ros message "draw links"
         */
        void drawLink();

        /*!
         * @brief publish ros message "activate motion planning"
         */
        void activateMP();

    private:
        void createTextEdits();
        void createLineEdits();
        void createPushButtons();

        geometry_msgs::Point scale(geometry_msgs::Point p, float sf);

//signals:
//    // change mode, send parameters

    protected:
        QLineEdit* lineEdit_seqFile_; /*!< One-line text editor for entering the outgoing ROS topic name. */

        QTextEdit* textEdit_ptDisplay_; /*!< Show the points */

        QPushButton* pushButton_browse_;
        QPushButton* pushButton_publishLink_;
        QPushButton* pushButton_startPlan_;

        QString file_name_; /*!< The current filename in the field  */

    private:
        //! ROS nodehandle
        //ros::NodeHandle nodeHandle_;

        //! ROS publisher
        ros::Publisher publisher_pose_;
        ros::Publisher publisher_motionPlan_;

        // todo: should separate data from this visualiztion class
        std::vector<geometry_msgs::Point> nodes_;
        std::deque<std::pair<int,int> >   edges_;
        std::vector<std::pair<int,int> >  pillars_;
        std::vector<std::pair<int,int> >  ceilings_;
};
}
#endif // FRAMEFAB_PANEL_H