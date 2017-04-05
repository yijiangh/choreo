#ifndef FRAMEFAB_PANEL_H
#define FRAMEFAB_PANEL_H

#include <ros/ros.h>

#include <QLineEdit>
#include <QPushButton>
#include <QTextEdit>

#include <rviz/panel.h>

namespace framefab {

class FramefabPanel: public rviz::Panel
{
// This class uses Qt slots and is a subclass of QObject, so it needs
// the Q_OBJECT macro.
Q_OBJECT
public:
  // QWidget subclass constructors usually take a parent widget
  // parameter (which usually defaults to 0).  At the same time,
  // pluginlib::ClassLoader creates instances by calling the default
  // constructor (with no arguments).  Taking the parameter and giving
  // a default of 0 lets the default constructor work and also lets
  // someone using the class for something else to pass in a parent
  // widget as they normally would with Qt.
  FramefabPanel( QWidget* parent = 0 );

  // Now we declare overrides of rviz::Panel functions for saving and
  // loading data from the config file.  Here the data is the
  // topic name.
  virtual void load( const rviz::Config& config );
  virtual void save( rviz::Config config ) const;

public Q_SLOTS:  
  void readFile();
  void drawLink();
	void activateMP();

protected:
	
	/* --- widget --- */
  // One-line text editor for entering the outgoing ROS topic name.
  QLineEdit* seq_file_editor_;

  //Show the points 
  QTextEdit* file_display_;

  QPushButton* browse_button_;
  QPushButton* publishlink_button_;
  QPushButton* startplan_button_;

  QString file_name_;						  // The current filename in the field
  
	/* --- publisher --- */
  ros::Publisher pose_publisher_;
	ros::Publisher mplan_publisher_;

	/* --- node handle --- */
  ros::NodeHandle nh_;
};
}
#endif // FRAMEFAB_PANEL_H
