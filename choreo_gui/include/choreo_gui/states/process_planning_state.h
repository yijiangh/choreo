//
// Created by yijiangh on 6/22/17.
//

#ifndef CHOREO_GUI_PROCESS_PLANNING_STATE_H
#define CHOREO_GUI_PROCESS_PLANNING_STATE_H

#include "choreo_gui/gui_state.h"
#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <choreo_msgs/ProcessPlanningAction.h>

namespace choreo_gui
{

class ProcessPlanningState : public GuiState
{
  Q_OBJECT
 public:
  // Constructor
  ProcessPlanningState(const int index, const bool use_ladder_graph_record);

  // Entry and exit classes
  virtual void onStart(ChoreoWidget& gui);
  virtual void onExit(ChoreoWidget& gui);

  // Handlers for the fixed buttons
  virtual void onNext(ChoreoWidget& gui);
  virtual void onBack(ChoreoWidget& gui);
  virtual void onReset(ChoreoWidget& gui);

 private:
  void makeRequest();
  void processPlanningDoneCallback(
      const actionlib::SimpleClientGoalState& state,
      const choreo_msgs::ProcessPlanningResultConstPtr& result);
  void processPlanningActiveCallback();
  void processPlanningFeedbackCallback(
      const choreo_msgs::ProcessPlanningFeedbackConstPtr& feedback);

  Q_SIGNALS:
  void feedbackReceived(QString feedback);

 private Q_SLOTS:
  void setFeedbackText(QString feedback);

 private:
  int selected_path_index_;
  bool use_ladder_graph_record_;

  actionlib::SimpleActionClient<choreo_msgs::ProcessPlanningAction> process_planning_action_client_;
  ChoreoWidget* gui_ptr_;
};
}

#endif //CHOREO_GUI_PROCESS_PLANNING_STATE_H
