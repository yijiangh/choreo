//
// Created by yijiangh on 6/28/17.
//
//#include "choreo_gui/states/scanning_state.h"

#include <ros/console.h>
#include <choreo_gui/states/select_tasks_state.h>
#include <choreo_gui/states/system_init_state.h>
#include <choreo_gui/states/task_sequence_processing_state.h>
#include <choreo_gui/states/process_planning_state.h>
//#include <QtConcurrent/QtConcurrentRun>

void choreo_gui::SelectTasksState::onStart(ChoreoWidget& gui)
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

void choreo_gui::SelectTasksState::onExit(ChoreoWidget& gui) {}

void choreo_gui::SelectTasksState::onNext(ChoreoWidget& gui)
{
  gui.setButtonsEnabled(false);

  // fetch ids (slider or input lineedit value from user) for planning from selection_widget
  int selected_id_for_planning = gui.selection_widget().getSelectedValueForPlanning();

  // fetch use_record decision (push_button choice from user) from selection_widget
  bool use_ladder_graph_record = gui.selection_widget().getUseSavedResult();

  // proceed to Process Planning State
  // params: selected_id_for_planning, (bool) use_record
  Q_EMIT newStateAvailable(new ProcessPlanningState(selected_id_for_planning, use_ladder_graph_record));
}

void choreo_gui::SelectTasksState::onBack(ChoreoWidget& gui)
{
  gui.selection_widget().cleanUpVisual();
  Q_EMIT newStateAvailable(new TaskSequenceProcessingState());
}

void choreo_gui::SelectTasksState::onReset(ChoreoWidget& gui)
{
  gui.selection_widget().cleanUpVisual();
  Q_EMIT newStateAvailable(new SystemInitState());
}

void choreo_gui::SelectTasksState::selectTask(ChoreoWidget& gui)
{
  gui.selection_widget().setMode(choreo_gui::SelectionWidget::PATH_SELECTION);

  // invoke the selection widget add waiting for a click on <close_widget>
  // and <Next> to move on to next state
  gui.selection_widget().show();
  gui.selection_widget().loadParameters();
}

void choreo_gui::SelectTasksState::toNextState()
{
  this->onNext(*ptr_gui_);
}

void choreo_gui::SelectTasksState::toBackState()
{
  this->onBack(*ptr_gui_);
}