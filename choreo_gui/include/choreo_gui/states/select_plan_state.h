//
// Created by yijiangh on 6/22/17.
//

#ifndef CHOREO_GUI_SELECT_PROCESS_PLAN_STATE_H
#define CHOREO_GUI_SELECT_PROCESS_PLAN_STATE_H

#include <choreo_gui/gui_state.h>
#include <choreo_gui/choreo_widget.h>
#include <ros/ros.h>

namespace choreo_gui
{

class SelectPlanState : public GuiState
{
  Q_OBJECT
 public:
  // Entry and exit classes
  virtual void onStart(ChoreoWidget& gui);
  virtual void onExit(ChoreoWidget& gui);

  // Handlers for the fixed buttons
  virtual void onNext(ChoreoWidget& gui);
  virtual void onBack(ChoreoWidget& gui);
  virtual void onReset(ChoreoWidget& gui);

 protected:
  void selectPlan(ChoreoWidget& gui);

 protected Q_SLOTS:
  void triggerSimulation();
  void triggerOutputProcess();

 private:
  void simulateAll();
  void simulateOne(const int& plan_id);

  void makeOutputProcessRequest();

 private:
  ChoreoWidget* ptr_gui_;

  std::vector<int> selected_plan_ids_;
  double sim_speed_;
};
}

#endif //CHOREO_GUI_SELECT_PROCESS_PLAN_STATE_H
