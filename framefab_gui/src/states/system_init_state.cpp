#include <framefab_gui/states/system_init_state.h>
#include <framefab_gui/states/task_sequence_processing_state.h>

#include <ros/console.h>
#include <framefab_gui/framefab_widget.h>

void framefab_gui::SystemInitState::onStart(FrameFabWidget& gui)
{
  gui.setText("System Init.\nClick <Params> to set model and path parameters.");
  gui.setParamsButtonEnabled(true);
}

void framefab_gui::SystemInitState::onExit(FrameFabWidget& gui) { gui.setParamsButtonEnabled(false); }

void framefab_gui::SystemInitState::onNext(FrameFabWidget& gui)
{
  Q_EMIT newStateAvailable(new TaskSequenceProcessingState());
}

void framefab_gui::SystemInitState::onBack(FrameFabWidget& gui) {}

void framefab_gui::SystemInitState::onReset(FrameFabWidget& gui) {}
