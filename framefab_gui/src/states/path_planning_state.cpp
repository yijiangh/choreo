//#include "framefab_gui/states/scanning_state.h"

#include <ros/console.h>
#include <framefab_gui/framefab_widget.h>
#include <framefab_gui/states/system_init_state.h>
#include <framefab_gui/states/path_planning_state.h>
#include <framefab_gui/states/select_path_state.h>
#include <QtConcurrent/QtConcurrentRun>

// input params
#include <framefab_msgs/ModelInputParameters.h>
#include <framefab_msgs/PathInputParameters.h>

framefab_gui::PathPlanningState::PathPlanningState()
    : task_sequence_processing_action_client_(TASK_SEQUENCE_PROCESSING_ACTION_CLIENT_NAME, true)
{
}

framefab_gui::PathPlanningState::~PathPlanningState()
{}

void framefab_gui::PathPlanningState::onStart(FrameFabWidget& gui)
{
  gui.setText("Task Sequence Processing State.\n"
                  "Please input data in parameter widget.\n"
                  "Click 'Next' to continue after finished.\n");
//  gui.setButtonsEnabled(false);

  gui_ptr_ = &gui;

  QObject::connect(this, SIGNAL(feedbackReceived(QString)), this, SLOT(setFeedbackText(QString)));
  QtConcurrent::run(this, &PathPlanningState::makeRequest, gui.params().modelInputParams(),
                    gui.params().pathInputParams());
}

void framefab_gui::PathPlanningState::onExit(FrameFabWidget& gui) { gui.setButtonsEnabled(true); }

void framefab_gui::PathPlanningState::onNext(FrameFabWidget& gui)
{
  Q_EMIT newStateAvailable(new SelectPathState());
}

void framefab_gui::PathPlanningState::onBack(FrameFabWidget& gui)
{
  gui.selection_widget().cleanUpVisual();
  Q_EMIT newStateAvailable(new SystemInitState());
}

void framefab_gui::PathPlanningState::onReset(FrameFabWidget& gui)
{
  gui.selection_widget().cleanUpVisual();
  Q_EMIT newStateAvailable(new SystemInitState());
}

void framefab_gui::PathPlanningState::makeRequest(
    framefab_msgs::ModelInputParameters model_params,
    framefab_msgs::PathInputParameters path_params)
{
  framefab_msgs::TaskSequenceProcessingGoal goal;
  goal.action = framefab_msgs::TaskSequenceProcessingGoal::FIND_AND_PROCESS;
  goal.use_default_parameters = false;
  goal.model_params = model_params;
  goal.path_params  = path_params;

//  ROS_INFO("Waiting for path planning action server to start.");
  task_sequence_processing_action_client_.waitForServer();
  if(task_sequence_processing_action_client_.isServerConnected())
  {
//    ROS_INFO_STREAM("path planning action server connected!");
  }
  else
  {
    ROS_ERROR_STREAM("action path planning server not connected");
  }

  task_sequence_processing_action_client_.sendGoal(
      goal,
      boost::bind(&framefab_gui::PathPlanningState::taskSequenceProcessingDoneCallback, this, _1, _2),
      boost::bind(&framefab_gui::PathPlanningState::taskSequenceProcessingActiveCallback, this),
      boost::bind(&framefab_gui::PathPlanningState::taskSequenceProcessingFeedbackCallback, this, _1));
}

void framefab_gui::PathPlanningState::setFeedbackText(QString feedback)
{
  gui_ptr_->appendText("-----------ACTION FEEDBACK-----------\n" + feedback.toStdString());
}

// Action Callbacks
void framefab_gui::PathPlanningState::taskSequenceProcessingDoneCallback(
    const actionlib::SimpleClientGoalState& state,
    const framefab_msgs::taskSequenceProcessingResultConstPtr& result)
{
  if(result->succeeded)
  {
    gui_ptr_->setButtonsEnabled(true);
  }
  else
  {
    Q_EMIT newStateAvailable(new SystemInitState());
  }
}

void framefab_gui::PathPlanningState::taskSequenceProcessingActiveCallback()
{
}

void framefab_gui::PathPlanningState::taskSequenceProcessingFeedbackCallback(
    const framefab_msgs::taskSequenceProcessingFeedbackConstPtr& feedback)
{
  Q_EMIT feedbackReceived(QString::fromStdString((feedback->last_completed).c_str()));
}
