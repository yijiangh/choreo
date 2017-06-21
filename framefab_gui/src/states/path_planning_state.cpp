//#include "framefab_gui/states/scanning_state.h"

#include <ros/console.h>
#include <framefab_gui/framefab_widget.h>
#include <framefab_gui/states/path_planning_state.h>
//#include <QtWidgets/QApplication>

framefab_gui::PathPlanningState::PathPlanningState()
{
  ROS_INFO_STREAM("PathPlanningState Init.");
}

framefab_gui::PathPlanningState::~PathPlanningState()
{
}

void framefab_gui::PathPlanningState::onStart(FrameFabWidget& gui)
{
  gui.setText("Input model. Click 'Next' to continue after finished.");
//  gui.setButtonsEnabled(false);

  gui.showInputUI(true);
}

void framefab_gui::PathPlanningState::onExit(FrameFabWidget& gui) {}

void framefab_gui::PathPlanningState::onNext(FrameFabWidget& gui)
{
  gui.showInputUI(false);
//  Q_EMIT newStateAvailable(new ScanningState());
}

void framefab_gui::PathPlanningState::onBack(FrameFabWidget& gui) {}

void framefab_gui::PathPlanningState::onReset(FrameFabWidget& gui) {}
