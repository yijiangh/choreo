//#include "framefab_gui/states/scanning_state.h"

#include <ros/console.h>
#include <framefab_gui/framefab_widget.h>
#include <framefab_gui/states/path_planning_state.h>
//#include <QtWidgets/QApplication>

framefab_gui::PathPlanningState::PathPlanningState()
    : ptr_input_mainwindow_(NULL)
{
  ROS_INFO_STREAM("PathPlanningState Init.");
}

framefab_gui::PathPlanningState::~PathPlanningState()
{
//  if(!ptr_input_mainwindow_)
//  {
//    delete ptr_input_mainwindow_;
//  }
}

void framefab_gui::PathPlanningState::onStart(FrameFabWidget& gui)
{
  gui.setText("Input model. Click 'Next' to continue after finished.");
  gui.setButtonsEnabled(false);

  ptr_input_mainwindow_ = new MainWindow;
  ptr_input_mainwindow_->setWindowFlags(Qt::Widget);
  ptr_input_mainwindow_->show();
}

void framefab_gui::PathPlanningState::onExit(FrameFabWidget& gui) {}

void framefab_gui::PathPlanningState::onNext(FrameFabWidget& gui)
{
//  Q_EMIT newStateAvailable(new ScanningState());
}

void framefab_gui::PathPlanningState::onBack(FrameFabWidget& gui) {}

void framefab_gui::PathPlanningState::onReset(FrameFabWidget& gui) {}
