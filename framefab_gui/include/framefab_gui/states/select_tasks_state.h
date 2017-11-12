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
  int selected_id_for_planning_;
  bool use_ladder_graph_record_;
  FrameFabWidget* ptr_gui_;
};
}

#endif //FRAMEFAB_GUI_SELECT_PATH_STATE_H