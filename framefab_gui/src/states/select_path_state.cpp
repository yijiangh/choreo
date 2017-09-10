//
// Created by yijiangh on 6/28/17.
//
//#include "framefab_gui/states/scanning_state.h"

#include <ros/console.h>
#include <framefab_gui/states/select_path_state.h>
#include <framefab_gui/states/system_init_state.h>
#include <framefab_gui/states/path_planning_state.h>
#include <framefab_gui/states/process_planning_state.h>
//#include <QtConcurrent/QtConcurrentRun>

void framefab_gui::SelectPathState::onStart(FrameFabWidget& gui)
{
  ptr_gui_ = &gui;

  gui.setText("Select Path State.\n"
                  "Select the desired path to be planned in selection window and click <select for plan>.\n"
                  "Close the selection widget to continue.");
  gui.setButtonsEnabled(false);
  selected_id_for_planning_ = -1;

  // if the selection widget is closed, move to next state
  connect(&gui.selection_widget(), SIGNAL(exitSelectionWidget()), this, SLOT(toNextState()));

  selectPath(gui);
}

void framefab_gui::SelectPathState::onExit(FrameFabWidget& gui) {}

void framefab_gui::SelectPathState::onNext(FrameFabWidget& gui)
{
  gui.setButtonsEnabled(false);

  // fetch ids for planning from selection_widget
  selected_id_for_planning_ = gui.selection_widget().getSelectedValueForPlanning();

//  gui.appendText("\nselect path state finished! Selected Path: #" + std::to_string(selected_path_ids_));
  Q_EMIT newStateAvailable(new ProcessPlanningState(selected_id_for_planning_));
}

void framefab_gui::SelectPathState::onBack(FrameFabWidget& gui)
{
  gui.selection_widget().cleanUpVisual();
  Q_EMIT newStateAvailable(new SystemInitState());
}

void framefab_gui::SelectPathState::onReset(FrameFabWidget& gui)
{
  gui.selection_widget().cleanUpVisual();
  Q_EMIT newStateAvailable(new SystemInitState());
}

void framefab_gui::SelectPathState::selectPath(FrameFabWidget& gui)
{
  gui.selection_widget().setMode(framefab_gui::SelectionWidget::PATH_SELECTION);
  gui.selection_widget().show();
  gui.selection_widget().loadParameters();
}

void framefab_gui::SelectPathState::toNextState()
{
  this->onNext(*ptr_gui_);
}