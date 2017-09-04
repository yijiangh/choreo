//
// Created by yijiangh on 6/22/17.
//

#ifndef FRAMEFAB_GUI_SIMULATING_STATE_H
#define FRAMEFAB_GUI_SIMULATING_STATE_H

#include "framefab_gui/gui_state.h"
#include <ros/ros.h>

// action
#include "actionlib/client/simple_action_client.h"

#include "framefab_msgs/SimulateMotionPlanAction.h"
#include "framefab_msgs/SimulateMotionPlanActionGoal.h"

namespace framefab_gui
{

class SimulatingState : public GuiState
{
  Q_OBJECT
 public:
  // Constructor
  SimulatingState(const std::vector<int>& plan_ids);

  // Entry and exit classes
  virtual void onStart(FrameFabWidget& gui);
  virtual void onExit(FrameFabWidget& gui);

  // Handlers for the fixed buttons
  virtual void onNext(FrameFabWidget& gui);
  virtual void onBack(FrameFabWidget& gui);
  virtual void onReset(FrameFabWidget& gui);

 protected:
  void simulateAll();
  void simulateOne(const int& plan_id);

  void simulateMotionPlanDoneCallback(
      const actionlib::SimpleClientGoalState& state,
      const framefab_msgs::SimulateMotionPlanResultConstPtr& result);
  void simulateMotionPlanActiveCallback();
  void simulateMotionPlanFeedbackCallback(
      const framefab_msgs::SimulateMotionPlanFeedbackConstPtr& feedback);

  Q_SIGNALS:
  void simulateFeedbackReceived(QString feedback);

 private Q_SLOTS:
  void setSimulateFeedbackText(QString feedback);

 private:
  std::vector<int> plan_ids_;
  FrameFabWidget* gui_ptr_;
  actionlib::SimpleActionClient<framefab_msgs::SimulateMotionPlanAction>
      simulate_motion_plan_action_client_;
};
}

#endif //FRAMEFAB_GUI_SIMULATING_STATE_H