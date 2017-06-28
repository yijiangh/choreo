//
// Created by yijiangh on 6/28/17.
//
//#include "framefab_gui/states/scanning_state.h"

#include <ros/console.h>
#include <framefab_gui/states/select_path_state.h>
#include <framefab_gui/states/system_init_state.h>
#include <framefab_gui/states/path_planning_state.h>
#include <QtConcurrent/QtConcurrentRun>

void framefab_gui::SelectPathState::onStart(FrameFabWidget& gui)
{
  gui.setText("PathPlanning State.\nPlease input data in parameter widget.\nClick 'Next' to continue after finished.");
  gui.setButtonsEnabled(false);

  QtConcurrent::run(this, &SelectPathState::selectPath, &gui);
}

void framefab_gui::SelectPathState::onExit(FrameFabWidget& gui) {}

void framefab_gui::SelectPathState::onNext(FrameFabWidget& gui)
{
  gui.setButtonsEnabled(true);
  gui.select_path().close();

  ROS_INFO_STREAM("select path state finished!");
//  Q_EMIT newStateAvailable(new ProcessPlanningState(std::vector<int> index_list));
}

void framefab_gui::SelectPathState::onBack(FrameFabWidget& gui)
{
  Q_EMIT newStateAvailable(new PathPlanningState());
}

void framefab_gui::SelectPathState::onReset(FrameFabWidget& gui)
{
  Q_EMIT newStateAvailable(new SystemInitState());
}

void framefab_gui::SelectPathState::selectPath(FrameFabWidget* ptr_gui)
{
  ptr_gui->select_path().loadParameters();
  ptr_gui->select_path().show();
}


