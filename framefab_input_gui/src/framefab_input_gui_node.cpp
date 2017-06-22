#include <ros/ros.h>

#include <GL/glut.h>
#include <QtWidgets/QApplication>

#include <framefab_input_gui/framefab_input_gui.h>
#include <framefab_input_gui/mainwindow.h>

int main(int argc, char** argv)
{
//  ros::init(argc, argv, "framefab_input_gui");
//  ros::AsyncSpinner spinner(4);

  // Load local parameters
//  ros::NodeHandle nh, pnh("~");
//  glutInit( & argc, argv );
  int i=0;
  glutInit(&i, NULL);
  QApplication a(argc, argv);
  MainWindow w;
  w.show();

//  using framefab_input_gui::FrameFabInputGui;

//  FrameFabInputGui input_gui(nh, &w);

  // Serve and wait for shutdown
//  ROS_INFO_STREAM("Framefab input gui service online");

//  spinner.start();
//  ros::spin();
  a.exec();

//  ros::waitForShutdown();

  return 0;
}
