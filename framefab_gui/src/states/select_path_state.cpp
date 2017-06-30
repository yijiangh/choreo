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
  gui.setText("Select Path State.\nPlease select the desired path to be planned in selection window.\nClick <Accept> to continue. ");
  gui.setButtonsEnabled(false);

  selectPath(gui);
//  QtConcurrent::run(this, &SelectPathState::selectPath, boost::ref(gui));
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
  gui.select_path().cleanUpVisual();
  Q_EMIT newStateAvailable(new SystemInitState());
}

void framefab_gui::SelectPathState::onReset(FrameFabWidget& gui)
{
  gui.select_path().cleanUpVisual();
  Q_EMIT newStateAvailable(new SystemInitState());
}

void framefab_gui::SelectPathState::selectPath(FrameFabWidget& gui)
{
  gui.select_path().show();
  gui.select_path().loadParameters();
}


