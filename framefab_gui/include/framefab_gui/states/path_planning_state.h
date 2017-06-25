#ifndef MODEL_INPUT_STATE_H
#define MODEL_INPUT_STATE_H

#include <ros/ros.h>
#include <framefab_gui/gui_state.h>

#include <actionlib/client/simple_action_client.h>
#include <framefab_msgs/ModelInputParameters.h>
#include <framefab_msgs/PathInputParameters.h>
#include <framefab_msgs/PathPlanningAction.h>
#include <framefab_msgs/FibonacciAction.h>

const static std::string PATH_PLANNING_ACTION_SERVER_NAME = "path_planning_as";

namespace framefab_gui
{

class PathPlanningState : public GuiState
{
  Q_OBJECT
 public:
  PathPlanningState();
  ~PathPlanningState();

 public:
  // Entry and exit classes
  virtual void onStart(FrameFabWidget& gui);
  virtual void onExit(FrameFabWidget& gui);

  // Handlers for the fixed buttons
  virtual void onNext(FrameFabWidget& gui);
  virtual void onBack(FrameFabWidget& gui);
  virtual void onReset(FrameFabWidget& gui);

 private:
  void pathPlanningDoneCallback(const actionlib::SimpleClientGoalState& state,
                                const framefab_msgs::PathPlanningResultConstPtr& result);
  void pathPlanningActiveCallback();
  void pathPlanningFeedbackCallback(const framefab_msgs::PathPlanningFeedbackConstPtr& feedback);

  void fDoneCallback(const actionlib::SimpleClientGoalState& state,
                     const framefab_msgs::FibonacciResultConstPtr& result);
  void fActiveCallback();
  void fFeedbackCallback(const framefab_msgs::FibonacciFeedbackConstPtr& feedback);

 private:
  void makeRequest(framefab_msgs::ModelInputParameters model_params,
                   framefab_msgs::PathInputParameters path_params);

  Q_SIGNALS:
  void feedbackReceived(QString feedback);

 protected Q_SLOTS:
  void setFeedbackText(QString feedback);

 private:
  ros::NodeHandle nh_;
  actionlib::SimpleActionClient<framefab_msgs::PathPlanningAction> path_planning_action_client_;

  actionlib::SimpleActionClient<framefab_msgs::FibonacciAction> f_ac_;

  FrameFabWidget* gui_ptr_;
};
}

#endif
