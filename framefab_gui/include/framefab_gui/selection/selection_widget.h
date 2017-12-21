//
// Created by yijiangh on 6/27/17.
//

#ifndef FRAMEFAB_GUI_SELECTION_WIDGET_H
#define FRAMEFAB_GUI_SELECTION_WIDGET_H

#include <QString>
#include <QWidget>

#include <ros/ros.h>

#include <framefab_gui/selection/select_for_plan_pop_up_widget.h>

namespace Ui
{
class SelectionWidgetWindow;
}

namespace framefab_gui
{

class SelectionWidget : public QWidget
{
 public:
  enum MODE
  {
    PATH_SELECTION,
    ZOOM_IN_SELECTION,
    PLAN_SELECTION
  };

  enum SIMULATE_TYPE
  {
    SINGLE,
    ALL_UNTIL,
    CHOSEN
  };

  Q_OBJECT
 public:
  SelectionWidget(QWidget* parent = 0);

  // set path or plan selection mode
  void setMode(MODE _mode) { mode_ = _mode; }
  void setModelFileName(std::string m) { model_file_name_ = m; }

  // service request on required parameters
  void loadParameters();

  // set max value, update slider & lineedit
  void setMaxValue(int m);

  // update display according to current print order
  void orderValueChanged();
  void simSpeedChanged();

  void setInputEnabled(bool enabled);
  void setInputIDEnabled(bool);
  void setInputLocaAxisEnabled(bool);
  void setInputIKSolutionEnabled(bool);

  int  getSelectedValueForPlanning() { return selected_value_; }
  double getSimSpeed() { return sim_speed_; }
  bool getUseSavedResult() { return use_saved_result_; }

  std::vector<int> getSelectedIdsForSimulation() { return selected_ids_for_sim_; }
  std::vector<int> getChosenIds() { return chosen_ids_for_sim_; }

  void setStatusBar(std::string string, bool state);

  SIMULATE_TYPE getSimulateType() { return sim_type_; }

  void addFetchedPlans(const std::vector<std::string> &plan_names);
  void getChosenPlans();

  // send srv to clean up visualization markers
  void cleanUpVisual();

  void showTaskSequenceRecomputePopUp(bool found_task_plan);

 protected:
  virtual void showEvent(QShowEvent *ev);
  virtual void closeEvent(QCloseEvent *ev);

 Q_SIGNALS:
  // this signal is for "pre-graph construction", after select plan id, enable zoom-in mode
//  void selectForPlan();

  void simulateOn(SIMULATE_TYPE sim_type);
//  void outputProcessOn(OUTPUT_TYPE output_type);
  void setOutputSaveDirOn();

  // to notify state in gui to start simulation
  void flushSimulation();
  void flushOutputProcess();

  void closeWidgetAndContinue();
  void enterSelectionWidget();
  void exitSelectionWidget();

 protected Q_SLOTS:
  // different source that changes order_value
  void buttonForwardUpdateOrderValue();
  void buttonBackwardUpdateOrderValue();
  void buttonSelectAll();

  void buttonSimulateSingle();
  void buttonSimulateUntil();
  void buttonSimulateChosen();
  void buttonSimulate(SIMULATE_TYPE sim_type);

  void buttonOutputChosen();

  void buttonClearSelection();

  void buttonCloseWidget();
  void buttonSelectForPlan();
  void sliderUpdateOrderValue(int value);
  void lineeditUpdateOrderValue();

  void sliderUpdateSimSpeed(int value);

  void recomputeChosen();
  void useSavedResultChosen();

  void recomputeTaskSequenceChosen();
  void useSavedTaskSequenceResultChosen();

 private:
  ros::NodeHandle nh_;

  ros::ServiceClient visualize_client_;
  ros::ServiceClient query_computation_record_client_;

  Ui::SelectionWidgetWindow* ui_;
  SelectForPlanPopUpWidget* select_for_plan_pop_up_;
  SelectForPlanPopUpWidget* task_seq_recompute_pop_up_;

  int max_value_;
  int selected_value_;
  std::vector<int> selected_ids_for_sim_;
  std::vector<int> chosen_ids_for_sim_;
  std::vector<int> fetched_plan_ids_;

  bool use_saved_result_;

  double sim_speed_;

  std::string model_file_name_;

  SIMULATE_TYPE sim_type_;
  MODE mode_;
};
}

#endif //FRAMEFAB_GUI_SELECTION_WIDGET_H
