//
// Created by yijiangh on 6/22/17.
//

#ifndef CHOREO_GUI_SELECT_PATH_STATE_H
#define CHOREO_GUI_SELECT_PATH_STATE_H

#include <choreo_gui/gui_state.h>
#include <choreo_gui/choreo_widget.h>
#include <ros/ros.h>

namespace choreo_gui
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
  virtual void onStart(ChoreoWidget& gui);
  virtual void onExit(ChoreoWidget& gui);

  // Handlers for the fixed buttons
  virtual void onNext(ChoreoWidget& gui);
  virtual void onBack(ChoreoWidget& gui);
  virtual void onReset(ChoreoWidget& gui);

 protected Q_SLOTS:
  void toBackState();
  void toNextState();

 protected:
  void selectTask(ChoreoWidget& gui);

 private:
  ChoreoWidget* ptr_gui_;
};
}

#endif //CHOREO_GUI_SELECT_PATH_STATE_H