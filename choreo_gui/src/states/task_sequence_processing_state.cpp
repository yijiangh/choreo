//#include "choreo_gui/states/scanning_state.h"

#include <ros/console.h>
#include <choreo_gui/choreo_widget.h>
#include <choreo_gui/states/system_init_state.h>
#include <choreo_gui/states/task_sequence_processing_state.h>
#include <choreo_gui/states/select_tasks_state.h>
#include <QtConcurrent/QtConcurrentRun>

// input params
#include <choreo_msgs/ModelInputParameters.h>
#include <choreo_msgs/TaskSequenceInputParameters.h>

// note: this action client's name MUST be the same to server's name
const static std::string TASK_SEQUENCE_PROCESSING_ACTION_CLIENT_NAME = "task_sequence_processing_action";
const static std::string TASK_SEQUENCE_PLANNING_ACTION_CLIENT_NAME = "task_sequence_planning_action";

choreo_gui::TaskSequenceProcessingState::TaskSequenceProcessingState()
    : task_sequence_processing_action_client_(TASK_SEQUENCE_PROCESSING_ACTION_CLIENT_NAME, true),
      task_sequence_planning_action_client_(TASK_SEQUENCE_PLANNING_ACTION_CLIENT_NAME, true)
{
}

choreo_gui::TaskSequenceProcessingState::~TaskSequenceProcessingState()
{}

void choreo_gui::TaskSequenceProcessingState::onStart(ChoreoWidget& gui)
{
  gui.setText("Task Sequence Processing State (recompute assembly sequence or read a saved one).\n"
                  "Click 'Next' to continue.\n");
//  gui.setButtonsEnabled(false);

  gui_ptr_ = &gui;

  QObject::connect(this, SIGNAL(feedbackReceived(QString)), this, SLOT(setFeedbackText(QString)));
  QObject::connect(&gui.selection_widget(), SIGNAL(closeWidgetAndContinue()), this, SLOT(toNextState()));
  QObject::connect(&gui.selection_widget(), SIGNAL(recomputeTaskSequenceChosen()), this, SLOT(taskSequencePlanningOn()));

  taskSequenceProcessOrPlan();
}

void choreo_gui::TaskSequenceProcessingState::onExit(ChoreoWidget& gui) { gui.setButtonsEnabled(true); }

void choreo_gui::TaskSequenceProcessingState::onNext(ChoreoWidget& gui)
{
  Q_EMIT newStateAvailable(new SelectTasksState());
}

void choreo_gui::TaskSequenceProcessingState::onBack(ChoreoWidget& gui)
{
  gui.selection_widget().cleanUpVisual();
  Q_EMIT newStateAvailable(new SystemInitState());
}

void choreo_gui::TaskSequenceProcessingState::onReset(ChoreoWidget& gui)
{
  gui.selection_widget().cleanUpVisual();
  Q_EMIT newStateAvailable(new SystemInitState());
}

void choreo_gui::TaskSequenceProcessingState::toNextState()
{
  this->onNext(*gui_ptr_);
}

void choreo_gui::TaskSequenceProcessingState::taskSequenceProcessOrPlan()
{
  gui_ptr_->setButtonsEnabled(false);

  std::string assembly_type;
  // it will first try to call tasks sequence processor to see if we can read any existing sequence result
  bool found_task_seq = makeTaskSequenceProcessingRequest(gui_ptr_->params().modelInputParams(),
                                       gui_ptr_->params().taskSequenceInputParams(), assembly_type);

  gui_ptr_->selection_widget().setAssemblyType(assembly_type);
  gui_ptr_->selection_widget().showTaskSequenceRecomputePopUp(found_task_seq);
}

bool choreo_gui::TaskSequenceProcessingState::makeTaskSequenceProcessingRequest(
    const choreo_msgs::ModelInputParameters& model_params,
    const choreo_msgs::TaskSequenceInputParameters& task_sequence_params,
    std::string& assembly_type)
{
  choreo_msgs::TaskSequenceProcessingGoal goal;
  goal.action = choreo_msgs::TaskSequenceProcessingGoal::FIND_AND_PROCESS;
  goal.use_default_parameters = false;
  goal.model_params = model_params;
  goal.task_sequence_params  = task_sequence_params;

  task_sequence_processing_action_client_.waitForServer();
  if(task_sequence_processing_action_client_.isServerConnected())
  {
//    ROS_INFO_STREAM("[UI] task sequence processing action server connected!");
  }
  else
  {
    ROS_ERROR_STREAM("[UI] action task sequence processing server not connected");
  }

  task_sequence_processing_action_client_.sendGoal(
      goal,
      boost::bind(&choreo_gui::TaskSequenceProcessingState::taskSequenceProcessingDoneCallback, this, _1, _2),
      boost::bind(&choreo_gui::TaskSequenceProcessingState::taskSequenceProcessingActiveCallback, this),
      boost::bind(&choreo_gui::TaskSequenceProcessingState::taskSequenceProcessingFeedbackCallback, this, _1));

  task_sequence_processing_action_client_.waitForResult();

  if(task_sequence_processing_action_client_.getResult()->succeeded)
  {
    assembly_type = task_sequence_processing_action_client_.getResult()->assembly_type;
    return true;
  }
  else
  {
    assembly_type.clear();
    return false;
  }
}

void choreo_gui::TaskSequenceProcessingState::setFeedbackText(QString feedback)
{
  gui_ptr_->appendText(feedback.toStdString());
}

// Action Callbacks
void choreo_gui::TaskSequenceProcessingState::taskSequenceProcessingDoneCallback(
    const actionlib::SimpleClientGoalState& state,
    const choreo_msgs::TaskSequenceProcessingResultConstPtr& result)
{
}

void choreo_gui::TaskSequenceProcessingState::taskSequenceProcessingActiveCallback()
{
}

void choreo_gui::TaskSequenceProcessingState::taskSequenceProcessingFeedbackCallback(
    const choreo_msgs::TaskSequenceProcessingFeedbackConstPtr& feedback)
{
  Q_EMIT feedbackReceived(QString::fromStdString((feedback->last_completed).c_str()));
}

void choreo_gui::TaskSequenceProcessingState::taskSequencePlanningOn()
{
  gui_ptr_->setButtonsEnabled(false);
  QtConcurrent::run(this, &TaskSequenceProcessingState::makeTaskSequencePlanningRequest,
                    gui_ptr_->params().modelInputParams(),
                    gui_ptr_->params().taskSequenceInputParams());
}

bool choreo_gui::TaskSequenceProcessingState::makeTaskSequencePlanningRequest(
    const choreo_msgs::ModelInputParameters& model_params,
    const choreo_msgs::TaskSequenceInputParameters& task_sequence_params)
{
  choreo_msgs::TaskSequencePlanningGoal goal;
  goal.model_params = model_params;
  goal.task_sequence_params = task_sequence_params;

  ROS_INFO("[UI] Waiting for task sequence planning action server to start.");
  task_sequence_planning_action_client_.waitForServer();
  if(task_sequence_planning_action_client_.isServerConnected())
  {
    ROS_INFO_STREAM("[UI] task seq planning action server connected!");
  }
  else
  {
    ROS_WARN_STREAM("[UI] action task seq planning server not connected");
  }

  task_sequence_planning_action_client_.sendGoal(
      goal,
      boost::bind(&choreo_gui::TaskSequenceProcessingState::taskSequencePlanningDoneCallback, this, _1, _2),
      boost::bind(&choreo_gui::TaskSequenceProcessingState::taskSequencePlanningActiveCallback, this),
      boost::bind(&choreo_gui::TaskSequenceProcessingState::taskSequencePlanningFeedbackCallback, this, _1));
  ROS_INFO_STREAM("[UI] Goal sent from task sequence planning state");
}

// Action Callbacks
void choreo_gui::TaskSequenceProcessingState::taskSequencePlanningDoneCallback(
    const actionlib::SimpleClientGoalState& state,
    const choreo_msgs::TaskSequencePlanningResultConstPtr& result)
{
  std::string assembly_type;

  if (result->succeeded && makeTaskSequenceProcessingRequest(gui_ptr_->params().modelInputParams(),
                                                             gui_ptr_->params().taskSequenceInputParams(),
                                                             assembly_type))
  {
    gui_ptr_->setButtonsEnabled(true);
  }
}

void choreo_gui::TaskSequenceProcessingState::taskSequencePlanningActiveCallback()
{
}

void choreo_gui::TaskSequenceProcessingState::taskSequencePlanningFeedbackCallback(
    const choreo_msgs::TaskSequencePlanningFeedbackConstPtr& feedback)
{
  Q_EMIT feedbackReceived(QString::fromStdString((feedback->last_completed).c_str()));
}