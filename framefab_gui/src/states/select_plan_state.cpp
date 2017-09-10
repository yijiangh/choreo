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
#include <QtConcurrent/QtConcurrentRun>

void framefab_gui::SelectPlanState::onStart(FrameFabWidget& gui)
{
  ptr_gui_ = &gui;

  gui.setText("Select Plan State.\nPlease select the desired plan to be simulated in selection window.\nClick <Simulate> to continue. ");
  gui.setButtonsEnabled(false);
  plan_ids_.clear();

  connect(&gui.selection_widget(), SIGNAL(flushSimulation()), this, SLOT(triggerSimulation()));

  selectPlan(gui);
}

void framefab_gui::SelectPlanState::onExit(FrameFabWidget& gui) {}

void framefab_gui::SelectPlanState::onNext(FrameFabWidget& gui) {}

void framefab_gui::SelectPlanState::onBack(FrameFabWidget& gui)
{
  gui.selection_widget().cleanUpVisual();
  Q_EMIT newStateAvailable(new SystemInitState());
}

void framefab_gui::SelectPlanState::onReset(FrameFabWidget& gui)
{
  gui.selection_widget().cleanUpVisual();
  Q_EMIT newStateAvailable(new SystemInitState());
}

void framefab_gui::SelectPlanState::selectPlan(FrameFabWidget& gui)
{
  gui.selection_widget().setMode(framefab_gui::SelectionWidget::PLAN_SELECTION);
  gui.selection_widget().show();
  gui.selection_widget().loadParameters();
}

void framefab_gui::SelectPlanState::triggerSimulation()
{
  QtConcurrent::run(this, &SelectPlanState::simulateAll);
}

void framefab_gui::SelectPlanState::simulateAll()
{
  plan_ids_.clear();
  plan_ids_ = ptr_gui_->selection_widget().getSelectedIdsForSimulation();

  for (std::size_t i = 0; i < plan_ids_.size(); ++i)
  {
    simulateOne(plan_ids_[i]);
    ROS_INFO_STREAM("[ui] simulate #" << i);
  }

  ptr_gui_->selection_widget().setInputEnabled(true);
}

void framefab_gui::SelectPlanState::simulateOne(const int& plan_id)
{
  framefab_msgs::SimulateMotionPlanGoal goal;
  goal.index = plan_id;
  goal.simulate = false;
  goal.wait_for_execution = false;

  ptr_gui_->sendGoalAndWait(goal);
}

