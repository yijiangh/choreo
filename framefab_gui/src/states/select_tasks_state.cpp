//
// Created by yijiangh on 6/28/17.
//
//#include "framefab_gui/states/scanning_state.h"

#include <ros/console.h>
#include <framefab_gui/states/select_tasks_state.h>
#include <framefab_gui/states/system_init_state.h>
#include <framefab_gui/states/task_sequence_processing_state.h>
#include <framefab_gui/states/process_planning_state.h>
//#include <QtConcurrent/QtConcurrentRun>

void framefab_gui::SelectTasksState::onStart(FrameFabWidget& gui)
{
  ptr_gui_ = &gui;

  gui.setText("Select Path State.\n"
                  "Select the desired path to be planned in selection window and click <select for plan>.\n"
                  "Close the selection widget to continue.");
  gui.setButtonsEnabled(false);
  selected_id_for_planning_ = -1;
  use_ladder_graph_record_ = false;

  // request checking ladder graph in record
//  connect(&gui.selection_widget(), SIGNAL(exitSelectionWidget()), this, SLOT(toNextState()));

  // if the selection widget is closed, move to next state
  connect(&gui.selection_widget(), SIGNAL(exitSelectionWidget()), this, SLOT(toBackState()));

  selectTask(gui);
}

void framefab_gui::SelectTasksState::onExit(FrameFabWidget& gui) {}

void framefab_gui::SelectTasksState::onNext(FrameFabWidget& gui)
{
  gui.setButtonsEnabled(false);

  // fetch ids for planning from selection_widget
  selected_id_for_planning_ = gui.selection_widget().getSelectedValueForPlanning();

  // fetch use_record decision from selection_widget
//  use_ladder_graph_record_ = gui.selection_widget().getUseLadderGraphRecord();

  // proceed to Process Planning State
  // params: selected_id_for_planning, (bool) use_record
  Q_EMIT newStateAvailable(new ProcessPlanningState(selected_id_for_planning_, use_ladder_graph_record_));
}

void framefab_gui::SelectTasksState::onBack(FrameFabWidget& gui)
{
  gui.selection_widget().cleanUpVisual();
  Q_EMIT newStateAvailable(new TaskSequenceProcessingState());
}

void framefab_gui::SelectTasksState::onReset(FrameFabWidget& gui)
{
  gui.selection_widget().cleanUpVisual();
  Q_EMIT newStateAvailable(new SystemInitState());
}

void framefab_gui::SelectTasksState::selectTask(FrameFabWidget& gui)
{
  gui.selection_widget().setMode(framefab_gui::SelectionWidget::PATH_SELECTION);
  gui.selection_widget().show();
  gui.selection_widget().loadParameters();
}

void framefab_gui::SelectTasksState::toNextState()
{
  this->onNext(*ptr_gui_);
}

void framefab_gui::SelectTasksState::toBackState()
{
  this->onBack(*ptr_gui_);
}