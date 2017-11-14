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

  std::string full_file_name = gui.params().taskSequenceInputParams().file_path;
  std::replace(full_file_name.begin(), full_file_name.end(), '/', '_');
  gui.selection_widget().setModelFileName(full_file_name);

  connect(&gui.selection_widget(), SIGNAL(closeWidgetAndContinue()), this, SLOT(toNextState()));
//  connect(&gui.selection_widget(), SIGNAL(exitSelectionWidget()), this, SLOT(toBackState()));

  selectTask(gui);
}

void framefab_gui::SelectTasksState::onExit(FrameFabWidget& gui) {}

void framefab_gui::SelectTasksState::onNext(FrameFabWidget& gui)
{
  gui.setButtonsEnabled(false);

  // fetch ids for planning from selection_widget
  int selected_id_for_planning = gui.selection_widget().getSelectedValueForPlanning();

  // fetch use_record decision from selection_widget
  bool use_ladder_graph_record = gui.selection_widget().getUseSavedResult();

  // proceed to Process Planning State
  // params: selected_id_for_planning, (bool) use_record
  Q_EMIT newStateAvailable(new ProcessPlanningState(selected_id_for_planning, use_ladder_graph_record));
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