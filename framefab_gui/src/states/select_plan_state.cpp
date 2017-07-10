//
// Created by yijiangh on 7/10/17.
//

//
// Created by yijiangh on 6/28/17.
//
//#include "framefab_gui/states/scanning_state.h"

#include <ros/console.h>
#include <framefab_gui/states/select_plan_state.h>
#include <framefab_gui/states/system_init_state.h>
//#include <QtConcurrent/QtConcurrentRun>

void framefab_gui::SelectPlanState::onStart(FrameFabWidget& gui)
{
  gui.setText("Select Plan State.\nPlease select the desired plan to be simulated in selection window.\nClick <Simulate> to continue. ");
  gui.setButtonsEnabled(false);
  selected_plan_id_ = -1;

  selectPlan(gui);
}

void framefab_gui::SelectPlanState::onExit(FrameFabWidget& gui) {}

void framefab_gui::SelectPlanState::onNext(FrameFabWidget& gui)
{
  gui.setButtonsEnabled(true);
  selected_plan_id_ = gui.select_path().getSelectedValue();
  gui.select_path().close();

  gui.appendText("\nselect plan state finished! Selected Plan: #" + std::to_string(selected_plan_id_));
//  Q_EMIT newStateAvailable(new SimulatingState(selected_plan_id_));
}

void framefab_gui::SelectPlanState::onBack(FrameFabWidget& gui)
{
  gui.select_path().cleanUpVisual();
  Q_EMIT newStateAvailable(new SystemInitState());
}

void framefab_gui::SelectPlanState::onReset(FrameFabWidget& gui)
{
  gui.select_path().cleanUpVisual();
  Q_EMIT newStateAvailable(new SystemInitState());
}

void framefab_gui::SelectPlanState::selectPlan(FrameFabWidget& gui)
{
  gui.select_path().setMode(framefab_gui::SelectPathWidget::PLAN_SELECTION);
  gui.select_path().show();
  gui.select_path().loadParameters();
}


