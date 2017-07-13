//
// Created by yijiangh on 7/10/17.
//

#include <ros/console.h>
#include <QtConcurrent/QtConcurrentRun>

// services
#include "framefab_msgs/SimulateMotionPlanAction.h"

#include "framefab_gui/framefab_widget.h"
#include <framefab_gui/states/system_init_state.h>
#include "framefab_gui/states/simulating_state.h"
//#include "framefab_gui/states/wait_to_execute_state.h"
#include <iostream>

const static std::string SIMULATE_MOTION_PLAN_SERVICE = "simulate_motion_plan";

framefab_gui::SimulatingState::SimulatingState(const std::vector<int>& plan_ids)
    : plan_ids_(plan_ids)
{
}

void framefab_gui::SimulatingState::onStart(FrameFabWidget& gui)
{
  gui.setText("Simulate State.\nSimulation in progress...");
  gui.setButtonsEnabled(false);

  QtConcurrent::run(this, &SimulatingState::simulateAll, boost::ref(gui));
}

void framefab_gui::SimulatingState::onExit(FrameFabWidget& gui) { gui.setButtonsEnabled(true); }

// Handlers for the fixed buttons
void framefab_gui::SimulatingState::onNext(FrameFabWidget& gui) {}

void framefab_gui::SimulatingState::onBack(FrameFabWidget& gui)
{
  gui.selection_widget().cleanUpVisual();
  Q_EMIT newStateAvailable(new SystemInitState());
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
  }

  gui.setButtonsEnabled(true);
//  Q_EMIT newStateAvailable(new WaitToExecuteState(plan_names_));
}

void framefab_gui::SimulatingState::simulateOne(const int& plan_id, FrameFabWidget& gui)
{
  framefab_msgs::SimulateMotionPlanActionGoal goal;
  goal.goal.index = plan_id;
  goal.goal.simulate = false;
  goal.goal.wait_for_execution = true;
  gui.sendGoalAndWait(goal);
}