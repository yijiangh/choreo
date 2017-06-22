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
//  ros::AsyncSpinner spinner(4);
  boost::thread spin_thread(&spinThread);

  // Load local parameters
//  ros::NodeHandle nh, pnh("~");
//  glutInit( & argc, argv );
  int i=0;
  glutInit(&i, NULL);
  QApplication a(argc, argv);
  MainWindow w;
  w.show();
  a.exec();

//  using framefab_input_gui::FrameFabInputGui;

//  FrameFabInputGui input_gui(nh, &w);


//  ros::waitForShutdown();

  return 0;
}
