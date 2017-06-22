#include <ros/ros.h>

#include <GL/glut.h>
#include <QtWidgets/QApplication>

#include <framefab_input_gui/framefab_input_gui.h>
#include <framefab_input_gui/mainwindow.h>

#include <boost/thread.hpp>

void spinThread()
{
  ros::spin();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "framefab_input_gui");
  boost::thread spin_thread(&spinThread);

  // Load local parameters
//  ros::NodeHandle nh, pnh("~");
//  glutInit( & argc, argv );

  int i=0;
  glutInit(&i, NULL);
  QApplication a(argc, argv);

//  using framefab_input_gui::FrameFabInputGui;

  framefab_input_gui::FrameFabInputGui input_gui;
  input_gui.setApp(&a);

  input_gui.init();

  return a.exec();
}
