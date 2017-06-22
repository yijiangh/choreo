#ifndef FRAMEFAB_MODEL_INPUT_UI_H
#define FRAMEFAB_MODEL_INPUT_UI_H

#include <ros/ros.h>

#include <QtWidgets/QApplication>
//#include <framefab_msgs/ModelInputGui.h>
#include <framefab_input_gui/mainwindow.h>

#include <framefab_msgs/ModelInputGuiAction.h>
#include <actionlib/server/simple_action_server.h>

namespace framefab_input_gui
{

class FrameFabInputGui
{
public:
  FrameFabInputGui();

//  bool handleModelInputGui(framefab_msgs::ModelInputGui::Request& req,
//                           framefab_msgs::ModelInputGui::Response& res);

  void setApp(QApplication* ptr_qapp) { ptr_app_ = ptr_qapp; }

  void init();

 private:
  void modelInputGuiActionCallback(const framefab_msgs::ModelInputGuiGoalConstPtr &goal);

private:
  MainWindow*     ptr_mainwindow_;
  QApplication*   ptr_app_;

  ros::NodeHandle nh_;

  // Actions offered by this class
  actionlib::SimpleActionServer<framefab_msgs::ModelInputGuiAction> model_input_ac_;
};

}

#endif
