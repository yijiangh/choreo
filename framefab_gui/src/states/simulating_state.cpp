//
// Created by yijiangh on 7/10/17.
//

#include <ros/console.h>
#include <QtConcurrent/QtConcurrentRun>

#include "framefab_gui/framefab_widget.h"
#include <framefab_gui/states/system_init_state.h>
#include <framefab_gui/states/select_plan_state.h>
#include "framefab_gui/states/simulating_state.h"
#include <iostream>

framefab_gui::SimulatingState::SimulatingState(const std::vector<int>& plan_ids)
    : plan_ids_(plan_ids)
{
}

void framefab_gui::SimulatingState::onStart(FrameFabWidget& gui)
{
  gui.setText("Simulate State.\nSimulation in progress...");
  gui.appendText("\nClick <Next> for post processing.\nClick on <Back> to choose plan and simulate again.");
  gui.setButtonsEnabled(false);
  QtConcurrent::run(this, &SimulatingState::simulateAll, boost::ref(gui));
}

void framefab_gui::SimulatingState::onExit(FrameFabWidget& gui) { gui.setButtonsEnabled(true); }

// Handlers for the fixed buttons
void framefab_gui::SimulatingState::onNext(FrameFabWidget& gui) {}

void framefab_gui::SimulatingState::onBack(FrameFabWidget& gui)
{
  gui.selection_widget().cleanUpVisual();
  Q_EMIT newStateAvailable(new SelectPlanState());
}

void framefab_gui::SimulatingState::onReset(FrameFabWidget& gui)
{
  gui.selection_widget().cleanUpVisual();
  Q_EMIT newStateAvailable(new SystemInitState());
}

void framefab_gui::SimulatingState::simulateAll(FrameFabWidget& gui)
{
  for (std::size_t i = 0; i < plan_ids_.size(); ++i)
  {
    simulateOne(plan_ids_[i], gui);
    ROS_INFO_STREAM("[SimulatingState ui] simulate #" << i);
  }

  gui.setButtonsEnabled(true);
}

void framefab_gui::SimulatingState::simulateOne(const int& plan_id, FrameFabWidget& gui)
{
  framefab_msgs::SimulateMotionPlanGoal goal;
  goal.index = plan_id;
  goal.simulate = false;
  goal.wait_for_execution = false;

  gui.sendGoalAndWait(goal);
}