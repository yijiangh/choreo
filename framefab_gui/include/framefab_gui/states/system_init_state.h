#ifndef SYSTEM_INIT_STATE_H
#define SYSTEM_INIT_STATE_H

#include <ros/ros.h>
#include <framefab_gui/gui_state.h>

namespace framefab_gui
{

class SystemInitState : public GuiState
{
  Q_OBJECT
 public:
  // Entry and exit classes
  virtual void onStart(FrameFabWidget& gui);
  virtual void onExit(FrameFabWidget& gui);

  // Handlers for the fixed buttons
  virtual void onNext(FrameFabWidget& gui);
  virtual void onBack(FrameFabWidget& gui);
  virtual void onReset(FrameFabWidget& gui);
};
}

#endif // SYSTEM_INIT_STATE_H