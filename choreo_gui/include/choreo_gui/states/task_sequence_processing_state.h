#ifndef MODEL_INPUT_STATE_H
#define MODEL_INPUT_STATE_H

#include <ros/ros.h>
#include <choreo_gui/gui_state.h>

#include <actionlib/client/simple_action_client.h>
#include <choreo_msgs/ModelInputParameters.h>
#include <choreo_msgs/TaskSequenceInputParameters.h>
#include <choreo_msgs/TaskSequenceProcessingAction.h>
#include <choreo_msgs/TaskSequencePlanningAction.h>

namespace choreo_gui
{

// In this ui state, core node will try to check if there is a saved assembly sequence
// in the input task sequence file path
// Then, depending on the reading result, users are asked to choose between
// (1) use saved task sequence (if found) (2) call task sequence planner to recompute a new one
//
// TODO: the task sequence call is only available for spatial printing task.
// general task sequence searching is still work in progress.
class TaskSequenceProcessingState : public GuiState
{
  Q_OBJECT
 public:
  TaskSequenceProcessingState();
  ~TaskSequenceProcessingState();

 public:
  // Entry and exit classes
  virtual void onStart(ChoreoWidget& gui);
  virtual void onExit(ChoreoWidget& gui);

  // Handlers for the fixed buttons
  virtual void onNext(ChoreoWidget& gui);
  virtual void onBack(ChoreoWidget& gui);
  virtual void onReset(ChoreoWidget& gui);

 private:
  void taskSequenceProcessingDoneCallback(const actionlib::SimpleClientGoalState& state,
                                const choreo_msgs::TaskSequenceProcessingResultConstPtr& result);
  void taskSequenceProcessingActiveCallback();
  void taskSequenceProcessingFeedbackCallback(const choreo_msgs::TaskSequenceProcessingFeedbackConstPtr& feedback);

  void taskSequencePlanningDoneCallback(const actionlib::SimpleClientGoalState& state,
                                        const choreo_msgs::TaskSequencePlanningResultConstPtr& result);
  void taskSequencePlanningActiveCallback();
  void taskSequencePlanningFeedbackCallback(const choreo_msgs::TaskSequencePlanningFeedbackConstPtr& feedback);

 private:
  void taskSequenceProcessOrPlan();
  bool makeTaskSequenceProcessingRequest(const choreo_msgs::ModelInputParameters& model_params,
                                         const choreo_msgs::TaskSequenceInputParameters& task_sequence_params,
                                         std::string& assembly_type);
  bool makeTaskSequencePlanningRequest(const choreo_msgs::ModelInputParameters& model_params,
                                       const choreo_msgs::TaskSequenceInputParameters& task_sequence_params);

  Q_SIGNALS:
  void feedbackReceived(QString feedback);

 protected Q_SLOTS:
  void setFeedbackText(QString feedback);
  void toNextState();
  void taskSequencePlanningOn();

 private:
  ros::NodeHandle nh_;
  actionlib::SimpleActionClient<choreo_msgs::TaskSequenceProcessingAction> task_sequence_processing_action_client_;
  actionlib::SimpleActionClient<choreo_msgs::TaskSequencePlanningAction> task_sequence_planning_action_client_;
  ChoreoWidget* gui_ptr_;
};
}

#endif
