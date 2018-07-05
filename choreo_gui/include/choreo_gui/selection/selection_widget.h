//
// Created by yijiangh on 6/27/17.
//

#ifndef CHOREO_GUI_SELECTION_WIDGET_H
#define CHOREO_GUI_SELECTION_WIDGET_H

#include <QString>
#include <QWidget>

#include <ros/ros.h>

#include <choreo_gui/selection/select_for_plan_pop_up_widget.h>

namespace Ui
{
class SelectionWidgetWindow;
}

namespace choreo_gui
{

class SelectionWidget : public QWidget
{
 public:
  enum MODE
  {
    PATH_SELECTION,
    ZOOM_IN_SELECTION, // TODO: DEPRECATED, should be merged with PATH SELECTION
    PLAN_SELECTION
  };

  enum ASSEMBLY_TYPE
  {
    SPATIAL_EXTRUSION,
    PICKNPLACE
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

  // CORE NODE COMMUNICATOR
  //
  // service request on required parameters
  // will trigger orderValueChanged to update visualization
  void loadParameters();

  // update display according to current print order
  // this function call core service to update visualization of assembly sequence
  // and is called whenever value changed in the selection widget ui panel
  // (slider, button, checkbox, etc., as long as it relates to visualization info)
  void orderValueChanged();

  // send srv to clean up visualization markers
  void cleanUpVisual();
  // CORE NODE COMMUNICATOR END
  //

  // STATE CHANGE RESPONSER
  //
  // mode-dependent all sub-widget switch
  void setInputEnabled(bool enabled);

  // switch for end effector visualization toggle
  void setInputEndEffectorVisualEnabled(bool);

  // switch for grasp selection related widget
  void setInputGraspEnabled(bool);

  // switch for IK selection related widget
  void setInputIKSolutionEnabled(bool);

  // TODO: not fully implemented
  void simSpeedChanged();
  // STATE CHANGE RESPONSER END
  //

  // GET FUNCTIONS
  //
  int  getSelectedValueForPlanning() const { return selected_value_; }
  double getSimSpeed() const { return sim_speed_; }
  bool getUseSavedResult() const { return use_saved_result_; }

  std::vector<int> getSelectedIdsForSimulation() { return selected_ids_for_sim_; }
  std::vector<int> getChosenIds() { return chosen_ids_for_sim_; }
  SIMULATE_TYPE getSimulateType() { return sim_type_; }
  //
  // GET FUNCTIONS END

  // INTERNAL DATA SET / GET FUNCTIONS
  //
  // set max value, update slider & lineedit
  void setMaxValue(int m);

  void setMaxGraspNum(int max_g);

  void setStatusBar(std::string string, bool state);

  // computed plan fetch / set / sort
  void addFetchedPlans(const std::vector<std::string> &plan_names);
  void getChosenPlans();

  // set path or plan selection mode
  void setMode(const MODE& _mode) { mode_ = _mode; }
  void setModelFileName(const std::string& m) { model_file_name_ = m; }

  void setAssemblyType(const std::string& at);
  //
  // INTERNAL DATA SET / GET FUNCTIONS

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
  void recomputeTaskSequenceChosen();
  void enableChoreoWidgetButtons();

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

  // slot function for qt pushbutton select for plan
  // change tab_widget to ZOOM_IN_SELECTION mode (tab 0)
  // call core node to query saved ladder graph
  // ask user to choose (1) use saved ladder graph and skip CLT-RRT*
  // or (2) recompute ladder graph
  void buttonSelectForPlan();

  // slot function for assembly order id update
  void sliderUpdateOrderValue(int value);
  void lineeditUpdateOrderValue();

  // slot function for grasp visual
  void checkboxEEVisualUpdateValue();

  // slot function for grasp visualization selection id update
  void sliderUpdateSelectedGraspValue(int value);
  void lineeditUpdateSelectedGraspValue();

  // slot function for sim speed update
  // TODO: not fully implemented
  void sliderUpdateSimSpeed(int value);

  // recompute popup selection slot function
  void recomputeChosen();
  void useSavedResultChosen();
  void useSavedTaskSequenceResultChosen();
  void popUpWindowClosed();

 private:
  ros::NodeHandle nh_;

  ros::ServiceClient visualize_client_;
  ros::ServiceClient query_computation_record_client_;

  Ui::SelectionWidgetWindow* ui_;
  SelectForPlanPopUpWidget* select_for_plan_pop_up_;
  SelectForPlanPopUpWidget* task_seq_recompute_pop_up_;

  // fetched id for slider and lineedit (depending on path or plan selection)
  int max_value_;

  // selected value for visualization
  // used to synchronize selected seq id's slider and linedit
  int selected_value_;

  // toggle for visualizing end effector
  bool visualize_ee_;

  // fetched grasp value for currect selection
  // Note: should be single value if in plan_selection mode
  std::vector<int> grasp_nums_;
  int max_grasp_num_;

  // selected value for grasp id for chosen assembly
  // used to synchronize selected grasp id's slider and lineedit
  int selected_grasp_id_;

  // TODO: should have max_ik_num & slected_ik_id_
  // To be implemented

  std::vector<int> selected_ids_for_sim_;
  std::vector<int> chosen_ids_for_sim_;
  std::vector<int> fetched_plan_ids_;

  // recompute or use saved result toggle
  // for (1) task sequence and (2) ladder graph
  bool use_saved_result_;

  // simulation speed for trajectory visualization
  // belongs to (0, 1.0]
  // TODO: not supported yet
  double sim_speed_;

  std::string model_file_name_;

  SIMULATE_TYPE sim_type_;
  MODE mode_;
  ASSEMBLY_TYPE assembly_type_;
};
}

#endif //CHOREO_GUI_SELECTION_WIDGET_H
