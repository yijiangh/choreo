#include <choreo_gui/states/system_init_state.h>
#include <choreo_gui/states/task_sequence_processing_state.h>

#include <ros/console.h>
#include <choreo_gui/choreo_widget.h>

void choreo_gui::SystemInitState::onStart(ChoreoWidget& gui)
{
  gui.setText("System Init.\nClick <Params> to set model and path parameters.");
  gui.setParamsButtonEnabled(true);
}

void choreo_gui::SystemInitState::onExit(ChoreoWidget& gui) { gui.setParamsButtonEnabled(false); }

void choreo_gui::SystemInitState::onNext(ChoreoWidget& gui)
{
  Q_EMIT newStateAvailable(new TaskSequenceProcessingState());
}

void choreo_gui::SystemInitState::onBack(ChoreoWidget& gui) {}

void choreo_gui::SystemInitState::onReset(ChoreoWidget& gui) {}
