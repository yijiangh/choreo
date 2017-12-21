//#include "framefab_gui/states/scanning_state.h"

#include <ros/console.h>
#include <framefab_gui/framefab_widget.h>
#include <framefab_gui/states/system_init_state.h>
#include <framefab_gui/states/task_sequence_processing_state.h>
#include <framefab_gui/states/select_tasks_state.h>
#include <QtConcurrent/QtConcurrentRun>

// input params
#include <framefab_msgs/ModelInputParameters.h>
#include <framefab_msgs/TaskSequenceInputParameters.h>

framefab_gui::TaskSequenceProcessingState::TaskSequenceProcessingState()
    : task_sequence_processing_action_client_(TASK_SEQUENCE_PROCESSING_ACTION_CLIENT_NAME, true)
{
}

framefab_gui::TaskSequenceProcessingState::~TaskSequenceProcessingState()
{}

void framefab_gui::TaskSequenceProcessingState::onStart(FrameFabWidget& gui)
{
  gui.setText("Task Sequence Processing State.\n"
                  "Please input data in parameter widget.\n"
                  "Click 'Next' to continue after finished.\n");
//  gui.setButtonsEnabled(false);

  gui_ptr_ = &gui;

  QObject::connect(this, SIGNAL(feedbackReceived(QString)), this, SLOT(setFeedbackText(QString)));
  QObject::connect(&gui.selection_widget(), SIGNAL(closeWidgetAndContinue()), this, SLOT(toNextState()));
//  QtConcurrent::run(this, &TaskSequenceProcessingState::makeRequest, gui.params().modelInputParams(),
//                    gui.params().taskSequenceInputParams());

  taskSequenceProcessOrPlan();
}

void framefab_gui::TaskSequenceProcessingState::onExit(FrameFabWidget& gui) { gui.setButtonsEnabled(true); }

void framefab_gui::TaskSequenceProcessingState::onNext(FrameFabWidget& gui)
{
  Q_EMIT newStateAvailable(new SelectTasksState());
}

void framefab_gui::TaskSequenceProcessingState::onBack(FrameFabWidget& gui)
{
  gui.selection_widget().cleanUpVisual();
  Q_EMIT newStateAvailable(new SystemInitState());
}

void framefab_gui::TaskSequenceProcessingState::onReset(FrameFabWidget& gui)
{
  gui.selection_widget().cleanUpVisual();
  Q_EMIT newStateAvailable(new SystemInitState());
}

void framefab_gui::TaskSequenceProcessingState::toNextState()
{
  this->onNext(*gui_ptr_);
}

void framefab_gui::TaskSequenceProcessingState::taskSequenceProcessOrPlan()
{
  // TODO: simplify this if else

  if(makeTaskSequenceProcessingRequest(gui_ptr_->params().modelInputParams(),
                                       gui_ptr_->params().taskSequenceInputParams()))
  {
    ROS_INFO_STREAM("UI - task seq plan found!");

    gui_ptr_->selection_widget().showTaskSequenceRecomputePopUp(true);
  }
  else
  {
    ROS_INFO_STREAM("UI - no task seq plan found!");
    gui_ptr_->selection_widget().showTaskSequenceRecomputePopUp(false);
  }
}

bool framefab_gui::TaskSequenceProcessingState::makeTaskSequenceProcessingRequest(
    framefab_msgs::ModelInputParameters model_params,
    framefab_msgs::TaskSequenceInputParameters task_sequence_params)
{
  framefab_msgs::TaskSequenceProcessingGoal goal;
  goal.action = framefab_msgs::TaskSequenceProcessingGoal::FIND_AND_PROCESS;
  goal.use_default_parameters = false;
  goal.model_params = model_params;
  goal.task_sequence_params  = task_sequence_params;

  task_sequence_processing_action_client_.waitForServer();
  if(task_sequence_processing_action_client_.isServerConnected())
  {
    ROS_INFO_STREAM("[UI] task sequence processing action server connected!");
  }
  else
  {
    ROS_ERROR_STREAM("[UI] action task sequence processing server not connected");
  }

  task_sequence_processing_action_client_.sendGoal(
      goal,
      boost::bind(&framefab_gui::TaskSequenceProcessingState::taskSequenceProcessingDoneCallback, this, _1, _2),
      boost::bind(&framefab_gui::TaskSequenceProcessingState::taskSequenceProcessingActiveCallback, this),
      boost::bind(&framefab_gui::TaskSequenceProcessingState::taskSequenceProcessingFeedbackCallback, this, _1));

  task_sequence_processing_action_client_.waitForResult();
  return task_sequence_processing_action_client_.getResult()->succeeded;
}

void framefab_gui::TaskSequenceProcessingState::setFeedbackText(QString feedback)
{
//  gui_ptr_->appendText("-----------ACTION FEEDBACK-----------\n" + feedback.toStdString());
}

// Action Callbacks
void framefab_gui::TaskSequenceProcessingState::taskSequenceProcessingDoneCallback(
    const actionlib::SimpleClientGoalState& state,
    const framefab_msgs::TaskSequenceProcessingResultConstPtr& result)
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

void framefab_gui::TaskSequenceProcessingState::taskSequenceProcessingActiveCallback()
{
}

void framefab_gui::TaskSequenceProcessingState::taskSequenceProcessingFeedbackCallback(
    const framefab_msgs::TaskSequenceProcessingFeedbackConstPtr& feedback)
{
  Q_EMIT feedbackReceived(QString::fromStdString((feedback->last_completed).c_str()));
}
