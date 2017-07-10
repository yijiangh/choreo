//
// Created by yijiangh on 6/22/17.
//

#ifndef FRAMEFAB_GUI_SIMULATING_STATE_H
#define FRAMEFAB_GUI_SIMULATING_STATE_H

#include "framefab_gui/gui_state.h"
#include <ros/ros.h>

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
  void simulateAll(FrameFabWidget& gui);
  void simulateOne(const int& plan_id, FrameFabWidget& gui);

 private:
  std::vector<int> plan_ids_;
};
}

#endif //FRAMEFAB_GUI_SIMULATING_STATE_H