//
// Created by yijiangh on 7/5/17.
//

#include <ros/console.h>
#include <QtConcurrent/QtConcurrentRun>
#include "framefab_gui/framefab_widget.h"
#include "framefab_gui/states/process_planning_state.h"  // previous
#include "framefab_gui/states/select_path_state.h"  // next if fail
//#include "framefab_gui/states/select_plans_state.h" // next is success

framefab_gui::ProcessPlanningState::ProcessPlanningState(const int& index)
    : process_planning_action_client_(PROCESS_PLANNING_ACTION_CLIENT_NAME, true),
      selected_path_index_(index)
{}

void framefab_gui::ProcessPlanningState::onStart(FrameFabWidget& gui)
{
  gui.setText("Process Planning...\n");
  gui.setButtonsEnabled(false);
  gui_ptr_ = &gui;
  QObject::connect(this, SIGNAL(feedbackReceived(QString)), this, SLOT(setFeedbackText(QString)));
  QtConcurrent::run(this, &ProcessPlanningState::makeRequest);
}

void framefab_gui::ProcessPlanningState::onExit(FrameFabWidget& gui) { gui.setButtonsEnabled(true); }

// Handlers for the fixed buttons
void framefab_gui::ProcessPlanningState::onNext(FrameFabWidget& gui) {}
void framefab_gui::ProcessPlanningState::onBack(FrameFabWidget& gui) {}
void framefab_gui::ProcessPlanningState::onReset(FrameFabWidget& gui) {}

// State Specific Functions
void framefab_gui::ProcessPlanningState::makeRequest()
{
  framefab_msgs::ProcessPlanningGoal goal;
  goal.action = framefab_msgs::ProcessPlanningGoal::GENERATE_MOTION_PLAN_AND_PREVIEW;
  goal.index = selected_path_index_;

  ROS_INFO("Waiting for process planning action server to start.");
  process_planning_action_client_.waitForServer();
  if(process_planning_action_client_.isServerConnected())
  {
    ROS_INFO_STREAM("process planning action server connected!");
  }
  else
  {
    ROS_WARN_STREAM("action process planning server not connected");
  }

  process_planning_action_client_.sendGoal(
      goal,
      boost::bind(&framefab_gui::ProcessPlanningState::processPlanningDoneCallback, this, _1, _2),
      boost::bind(&framefab_gui::ProcessPlanningState::processPlanningActiveCallback, this),
      boost::bind(&framefab_gui::ProcessPlanningState::processPlanningFeedbackCallback, this, _1));
  ROS_INFO_STREAM("Goal sent from process planning state");
}

void framefab_gui::ProcessPlanningState::setFeedbackText(QString feedback)
{
  gui_ptr_->appendText("-----------ACTION FEEDBACK-----------\n" + feedback.toStdString());
}

// Action Callbacks
void framefab_gui::ProcessPlanningState::processPlanningDoneCallback(
    const actionlib::SimpleClientGoalState& state,
    const framefab_msgs::ProcessPlanningResultConstPtr& result)
{
  if (result->succeeded)
  {
//    Q_EMIT newStateAvailable(new SelectPlansState());
  }
  else
  {
    Q_EMIT newStateAvailable(new SelectPathState());
  }
}

void framefab_gui::ProcessPlanningState::processPlanningActiveCallback()
{
  ROS_INFO_STREAM("Process Planning Goal is active");
}

void framefab_gui::ProcessPlanningState::processPlanningFeedbackCallback(
    const framefab_msgs::ProcessPlanningFeedbackConstPtr& feedback)
{
  Q_EMIT feedbackReceived(QString::fromStdString((feedback->last_completed).c_str()));
}
