//
// Created by yijiangh on 6/22/17.
//

#ifndef FRAMEFAB_GUI_SELECT_PATH_STATE_H
#define FRAMEFAB_GUI_SELECT_PATH_STATE_H

#include <framefab_gui/gui_state.h>
#include <framefab_gui/framefab_widget.h>
#include <ros/ros.h>

namespace framefab_gui
{

// in this state, based on the parsed (or freshly computed)
// sequence result, the ui will invoke selection widget to
// call core node to visualize assembly sequence and associated
// grasp poses. User can choose up until which index that she/he
// wants to compute the plan.
// In the light of upcoming support for picknplace, Choreo will
// start to support visualization for input grasps (end effector included)
//
class SelectTasksState : public GuiState
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

 protected Q_SLOTS:
  void toBackState();
  void toNextState();

 protected:
  void selectTask(FrameFabWidget& gui);

 private:
  FrameFabWidget* ptr_gui_;
};
}

#endif //FRAMEFAB_GUI_SELECT_PATH_STATE_H