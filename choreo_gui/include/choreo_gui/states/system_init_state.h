#ifndef SYSTEM_INIT_STATE_H
#define SYSTEM_INIT_STATE_H

#include <ros/ros.h>
#include <choreo_gui/gui_state.h>

namespace choreo_gui
{

class SystemInitState : public GuiState
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
};
}

#endif // SYSTEM_INIT_STATE_H