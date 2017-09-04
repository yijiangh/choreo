//
// Created by yijiangh on 7/10/17.
//

#include <ros/console.h>
#include <QtConcurrent/QtConcurrentRun>

// services
#include "framefab_msgs/SimulateMotionPlanAction.h"

#include "framefab_gui/framefab_widget.h"
#include <framefab_gui/states/system_init_state.h>
#include <framefab_gui/states/select_plan_state.h>
#include "framefab_gui/states/simulating_state.h"
//#include "framefab_gui/states/wait_to_execute_state.h"
#include <iostream>

const static std::string SIMULATE_MOTION_PLAN_ACTION_SERVER_NAME = "simulate_motion_plan_as";

framefab_gui::SimulatingState::SimulatingState(const std::vector<int>& plan_ids)
    : plan_ids_(plan_ids),
      simulate_motion_plan_action_client_(SIMULATE_MOTION_PLAN_ACTION_SERVER_NAME, true)
{
}

void framefab_gui::SimulatingState::onStart(FrameFabWidget& gui)
{
  gui.setText("Simulate State.\nSimulation in progress...");
  gui.setButtonsEnabled(false);
  gui_ptr_ = &gui;
  QObject::connect(this, SIGNAL(simulateFeedbackReceived(QString)), this, SLOT(setSimulateFeedbackText(QString)));
  QtConcurrent::run(this, &SimulatingState::simulateAll);
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

void framefab_gui::SimulatingState::simulateAll()
{
  for (std::size_t i = 0; i < plan_ids_.size(); ++i)
  {
    simulateOne(plan_ids_[i]);
    ROS_INFO_STREAM("[SimulatingState ui] simulate #" << i);
  }

  gui_ptr_->setButtonsEnabled(true);
  gui_ptr_->appendText("\nSimulation finished! Click on <Back> to choose plan and simulate again.");

//  Q_EMIT newStateAvailable(new WaitToExecuteState(plan_names_));
}

void framefab_gui::SimulatingState::simulateOne(const int& plan_id)
{
  framefab_msgs::SimulateMotionPlanGoal goal;
  goal.index = plan_id;
  goal.simulate = false;
  goal.wait_for_execution = false;

  ros::Duration timeout = ros::Duration(60);

  simulate_motion_plan_action_client_.sendGoal(goal);
}

void framefab_gui::SimulatingState::setSimulateFeedbackText(QString feedback)
{
  gui_ptr_->appendText("-----------ACTION FEEDBACK-----------\n" + feedback.toStdString() + "\n");
}

void framefab_gui::SimulatingState::simulateMotionPlanDoneCallback(
    const actionlib::SimpleClientGoalState& state,
    const framefab_msgs::SimulateMotionPlanResultConstPtr& result)
{
  switch (result->code)
  {
    case framefab_msgs::SimulateMotionPlanResult::SUCCESS:
    {
      Q_EMIT simulateFeedbackReceived(QString("Simulation success."));
      break;
    }
    case framefab_msgs::SimulateMotionPlanResult::NO_SUCH_NAME:
    {
      Q_EMIT simulateFeedbackReceived(QString("Failed to find plan."));
      break;
    }
    case framefab_msgs::SimulateMotionPlanResult::TIMEOUT:
    {
      Q_EMIT simulateFeedbackReceived(QString("Failed due to timeout"));
      break;
    }
    case framefab_msgs::SimulateMotionPlanResult::RESET_POSE_FAIL:
    {
      Q_EMIT simulateFeedbackReceived(QString("Reset pose to initial pose failed"));
      break;
    }
  }
}

void framefab_gui::SimulatingState::simulateMotionPlanActiveCallback()
{
  ROS_INFO_STREAM("Simulation Goal is active");
}

void framefab_gui::SimulatingState::simulateMotionPlanFeedbackCallback(
    const framefab_msgs::SimulateMotionPlanFeedbackConstPtr& feedback)
{
  Q_EMIT simulateFeedbackReceived(QString::fromStdString((feedback->last_completed).c_str()));
}