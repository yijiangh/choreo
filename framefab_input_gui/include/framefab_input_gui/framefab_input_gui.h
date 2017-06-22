#ifndef FRAMEFAB_MODEL_INPUT_UI_H
#define FRAMEFAB_MODEL_INPUT_UI_H

#include <ros/ros.h>

//#include <QtWidgets/QApplication>
#include <framefab_msgs/ModelInputGui.h>
#include <framefab_input_gui/mainwindow.h>

// Globals
const static std::string DEFAULT_INPUT_GUI_SERVICE = "model_input_gui";

namespace framefab_input_gui
{

class FrameFabInputGui
{
public:
  FrameFabInputGui(ros::NodeHandle node_handle, MainWindow* ptr_mainwindow);

  bool handleModelInputGui(framefab_msgs::ModelInputGui::Request& req,
                           framefab_msgs::ModelInputGui::Response& res);

private:
  MainWindow*     ptr_mainwindow_;
  ros::NodeHandle nh_;

  ros::ServiceServer input_ui_server_;
};
}

#endif
