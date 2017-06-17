#include <framefab_gui/states/model_input_state.h>
//#include "framefab_gui/states/scanning_state.h"

#include <ros/console.h>
#include <framefab_gui/framefab_widget.h>

void framefab_gui::ModelInputState::onStart(FrameFabWidget& gui)
{
  gui.setText("Input model. Click 'Next' to continue after finished.");
}

void framefab_gui::ModelInputState::onExit(FrameFabWidget& gui) {}

void framefab_gui::ModelInputState::onNext(FrameFabWidget& gui)
{
//  Q_EMIT newStateAvailable(new ScanningState());
}

void framefab_gui::ModelInputState::onBack(FrameFabWidget& gui) {}

void framefab_gui::ModelInputState::onReset(FrameFabWidget& gui) {}
