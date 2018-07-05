//
// Created by yijiangh on 7/5/17.
//

#include <ros/console.h>
#include <QtConcurrent/QtConcurrentRun>
#include "choreo_gui/choreo_widget.h"
#include "choreo_gui/states/system_init_state.h"
#include "choreo_gui/states/process_planning_state.h"  // previous
#include "choreo_gui/states/select_tasks_state.h"  // next if fail
#include "choreo_gui/states/select_plan_state.h" // next if success

const static std::string PROCESS_PLANNING_ACTION_CLIENT_NAME = "process_planning_action";

choreo_gui::ProcessPlanningState::ProcessPlanningState(const int index, const bool use_ladder_graph_record)
    : process_planning_action_client_(PROCESS_PLANNING_ACTION_CLIENT_NAME, true),
      selected_path_index_(index),
      use_ladder_graph_record_(use_ladder_graph_record)
{}

void choreo_gui::ProcessPlanningState::onStart(ChoreoWidget& gui)
{
  gui.setText("Process Planning...\n");
  gui.setButtonsEnabled(false);
  gui_ptr_ = &gui;
  QObject::connect(this, SIGNAL(feedbackReceived(QString)), this, SLOT(setFeedbackText(QString)));
  QtConcurrent::run(this, &ProcessPlanningState::makeRequest);
}

void choreo_gui::ProcessPlanningState::onExit(ChoreoWidget& gui) { gui.setButtonsEnabled(true); }

// Handlers for the fixed buttons
void choreo_gui::ProcessPlanningState::onNext(ChoreoWidget& gui)
{
  gui.selection_widget().cleanUpVisual();
  Q_EMIT newStateAvailable(new SelectPlanState());
}

void choreo_gui::ProcessPlanningState::onBack(ChoreoWidget& gui)
{
  gui.selection_widget().cleanUpVisual();
  Q_EMIT newStateAvailable(new SelectTasksState());
}

void choreo_gui::ProcessPlanningState::onReset(ChoreoWidget& gui)
{
  gui.selection_widget().cleanUpVisual();
  Q_EMIT newStateAvailable(new SystemInitState());
}

// State Specific Functions
void choreo_gui::ProcessPlanningState::makeRequest()
{
  choreo_msgs::ProcessPlanningGoal goal;
  goal.action = choreo_msgs::ProcessPlanningGoal::GENERATE_MOTION_PLAN_AND_PREVIEW;
  goal.index = selected_path_index_;
  goal.use_saved_graph = use_ladder_graph_record_;

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
      boost::bind(&choreo_gui::ProcessPlanningState::processPlanningDoneCallback, this, _1, _2),
      boost::bind(&choreo_gui::ProcessPlanningState::processPlanningActiveCallback, this),
      boost::bind(&choreo_gui::ProcessPlanningState::processPlanningFeedbackCallback, this, _1));
  ROS_INFO_STREAM("Goal sent from process planning state");
}

void choreo_gui::ProcessPlanningState::setFeedbackText(QString feedback)
{
  gui_ptr_->appendText(feedback.toStdString());
}

// Action Callbacks
void choreo_gui::ProcessPlanningState::processPlanningDoneCallback(
    const actionlib::SimpleClientGoalState& state,
    const choreo_msgs::ProcessPlanningResultConstPtr& result)
{
  if (result->succeeded)
  {
//    Q_EMIT newStateAvailable(new SelectPlansState());
    gui_ptr_->setButtonsEnabled(true);
  }
  else
  {
//    Q_EMIT newStateAvailable(new SelectTasksState());
  }
}

void choreo_gui::ProcessPlanningState::processPlanningActiveCallback()
{
  ROS_INFO_STREAM("Process Planning Goal is active");
}

void choreo_gui::ProcessPlanningState::processPlanningFeedbackCallback(
    const choreo_msgs::ProcessPlanningFeedbackConstPtr& feedback)
{
  Q_EMIT feedbackReceived(QString::fromStdString((feedback->last_completed).c_str()));
}
