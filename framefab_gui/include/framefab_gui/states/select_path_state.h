//
// Created by yijiangh on 6/22/17.
//

#ifndef FRAMEFAB_GUI_SELECT_PATH_STATE_H
#define FRAMEFAB_GUI_SELECT_PATH_STATE_H

#include "framefab_gui/gui_state.h"
#include <ros/ros.h>

namespace framefab_gui
{

class SelectPathState : public GuiState
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

 protected:
  void selectPath(FrameFabWidget& gui);

 private:

};
}

#endif //FRAMEFAB_GUI_SELECT_PATH_STATE_H
