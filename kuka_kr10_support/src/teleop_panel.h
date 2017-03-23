#ifndef TELEOP_PANEL_H
#define TELEOP_PANEL_H

#include <ros/ros.h>
#include <QLineEdit>
#include <QPushButton>
#include <QTextEdit>
#include <rviz/panel.h>


namespace framefab {

class TeleopPanel: public rviz::Panel
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
  TeleopPanel( QWidget* parent = 0 );

  // Now we declare overrides of rviz::Panel functions for saving and
  // loading data from the config file.  Here the data is the
  // topic name.
  virtual void load( const rviz::Config& config );
  virtual void save( rviz::Config config ) const;

  // Next come a couple of public Qt slots.
public Q_SLOTS:
  
  void readFile();
  // Here we declare some internal slots.
  // Then we finish up with protected member variables.
protected:
  // One-line text editor for entering the outgoing ROS topic name.
  QLineEdit* seq_file_editor_;
  QPushButton* button_;
  // The current filename in the field
  QString file_name_;
  
  //Show the points 
  QTextEdit* file_display_;

  ros::Publisher pose_publisher_;

  ros::NodeHandle nh_;

};
}
#endif // TELEOP_PANEL_H
