#ifndef MODEL_INPUT_STATE_H
#define MODEL_INPUT_STATE_H

#include <ros/ros.h>
#include <framefab_gui/gui_state.h>

#include <actionlib/client/simple_action_client.h>
#include <framefab_msgs/ModelInputParameters.h>
#include <framefab_msgs/TaskSequenceInputParameters.h>
#include <framefab_msgs/TaskSequenceProcessingAction.h>

// note: this action client's name MUST be the same to server's name
const static std::string TASK_SEQUENCE_PROCESSING_ACTION_CLIENT_NAME = "task_sequence_processing_action";

namespace framefab_gui
{

class TaskSequenceProcessingState : public GuiState
{
  Q_OBJECT
 public:
  TaskSequenceProcessingState();
  ~TaskSequenceProcessingState();

 public:
  // Entry and exit classes
  virtual void onStart(FrameFabWidget& gui);
  virtual void onExit(FrameFabWidget& gui);

  // Handlers for the fixed buttons
  virtual void onNext(FrameFabWidget& gui);
  virtual void onBack(FrameFabWidget& gui);
  virtual void onReset(FrameFabWidget& gui);

 private:
  void taskSequenceProcessingDoneCallback(const actionlib::SimpleClientGoalState& state,
                                const framefab_msgs::TaskSequenceProcessingResultConstPtr& result);
  void taskSequenceProcessingActiveCallback();
  void taskSequenceProcessingFeedbackCallback(const framefab_msgs::TaskSequenceProcessingFeedbackConstPtr& feedback);

 private:
  void makeRequest(framefab_msgs::ModelInputParameters model_params,
                   framefab_msgs::TaskSequenceInputParameters task_sequence_params);

  Q_SIGNALS:
  void feedbackReceived(QString feedback);

 protected Q_SLOTS:
  void setFeedbackText(QString feedback);

 private:
  ros::NodeHandle nh_;
  actionlib::SimpleActionClient<framefab_msgs::TaskSequenceProcessingAction> task_sequence_processing_action_client_;
  FrameFabWidget* gui_ptr_;
};
}

#endif
