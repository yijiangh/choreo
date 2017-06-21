#ifndef MODEL_INPUT_STATE_H
#define MODEL_INPUT_STATE_H

#include <ros/ros.h>
#include <framefab_gui/gui_state.h>

#include <framefab_gui/input_ui/mainwindow.h>

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
  MainWindow* ptr_input_mainwindow_;
};
}

#endif