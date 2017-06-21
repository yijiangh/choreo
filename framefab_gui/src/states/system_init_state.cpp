#include <framefab_gui/states/system_init_state.h>
#include <framefab_gui/states/path_planning_state.h>

#include <ros/console.h>
#include <framefab_gui/framefab_widget.h>

void framefab_gui::SystemInitState::onStart(FrameFabWidget& gui)
{
  gui.setText("System Ready. Click 'Next' to input model.");
}

void framefab_gui::SystemInitState::onExit(FrameFabWidget& gui) {}

void framefab_gui::SystemInitState::onNext(FrameFabWidget& gui)
{
  Q_EMIT newStateAvailable(new PathPlanningState());
}

void framefab_gui::SystemInitState::onBack(FrameFabWidget& gui) {}

void framefab_gui::SystemInitState::onReset(FrameFabWidget& gui) {}