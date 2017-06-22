//
// Created by yijiangh on 6/21/17.
//
#include <framefab_input_gui/framefab_input_gui.h>

// Globals
const static std::string MODEL_INPUT_GUI_ACTION_SERVER_NAME = "model_input_gui_server_ac";

framefab_input_gui::FrameFabInputGui::FrameFabInputGui()
  :   model_input_ac_(nh_, MODEL_INPUT_GUI_ACTION_SERVER_NAME,
                               boost::bind(&FrameFabInputGui::modelInputGuiActionCallback, this, _1), false)
{
}

void framefab_input_gui::FrameFabInputGui::init()
{
  ptr_mainwindow_ = new MainWindow();
  model_input_ac_.start();

}

void framefab_input_gui::FrameFabInputGui::modelInputGuiActionCallback(
    const framefab_msgs::ModelInputGuiGoalConstPtr &goal_in)
{
  switch (goal_in->enable_ui)
  {
    case true:
    {
      ROS_INFO_STREAM("show gui received!");
      ptr_mainwindow_->show();
      break;
    }
    case false:
    {
      ROS_INFO_STREAM("disable gui received!");
      ptr_mainwindow_->hide();
      model_input_ac_.setSucceeded();
      break;
    }
  }
}

//bool framefab_input_gui::FrameFabInputGui::handleModelInputGui(
//    framefab_msgs::ModelInputGui::Request& req, framefab_msgs::ModelInputGui::Response& res)
//{
//  ptr_mainwindow_->show();
////  ptr_qapp_->exec();
//
//  return true;
//}