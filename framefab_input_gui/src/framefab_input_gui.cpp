//
// Created by yijiangh on 6/21/17.
//
#include <framefab_input_gui/framefab_input_gui.h>

framefab_input_gui::FrameFabInputGui::FrameFabInputGui(ros::NodeHandle node_handle, MainWindow* ptr_mainwindow)
  : nh_(node_handle), ptr_mainwindow_(ptr_mainwindow)
{
  input_ui_server_ = nh_.advertiseService(
      DEFAULT_INPUT_GUI_SERVICE, &FrameFabInputGui::handleModelInputGui, this);
}

bool framefab_input_gui::FrameFabInputGui::handleModelInputGui(
    framefab_msgs::ModelInputGui::Request& req, framefab_msgs::ModelInputGui::Response& res)
{
  ptr_mainwindow_->show();
//  ptr_qapp_->exec();

  return true;
}