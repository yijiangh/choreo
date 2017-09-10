//
// Created by yijiangh on 6/27/17.
//

#ifndef FRAMEFAB_GUI_SELECTION_WIDGET_H
#define FRAMEFAB_GUI_SELECTION_WIDGET_H

#include <QString>
#include <QWidget>

#include <ros/ros.h>

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

  // service request on required parameters
  void loadParameters();

  // set max value, update slider & lineedit
  void setMaxValue(int m);

  // update display according to current print order
  void orderValueChanged();

  void setInputEnabled(bool enabled);
  void setInputIDEnabled(bool);
  void setInputLocaAxisEnabled(bool);
  void setInputIKSolutionEnabled(bool);

  int  getSelectedValueForPlanning() { return selected_value_; }
  std::vector<int> getSelectedIdsForSimulation() { return selected_ids_for_sim_; }
  SIMULATE_TYPE getSimulateType() { return sim_type_; }

  void addFetchedPlans(const std::vector<std::string> &plan_names);
  void getChosenPlans();

  // send srv to clean up visualization markers
  void cleanUpVisual();

 protected:
  virtual void showEvent(QShowEvent *ev);
  virtual void closeEvent(QCloseEvent *ev);

 Q_SIGNALS:
  // this signal is for "pre-graph construction", after select plan id, enable zoom-in mode
//  void selectForPlan();

  void simulateOn(SIMULATE_TYPE sim_type);

  // to notify state in gui to start simulation
  void flushSimulation();

//  void closeWidgetAndContinue();
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

  void buttonCloseWidget();
  void buttonSelectForPlan();
  void sliderUpdateOrderValue(int value);
  void lineeditUpdateOrderValue();

  void widgetStateChanged();

 private:
  ros::NodeHandle nh_;

  ros::ServiceClient visualize_client_;

  Ui::SelectionWidgetWindow* ui_;

  int max_value_;
  int selected_value_;
  std::vector<int> selected_ids_for_sim_;
  std::vector<int> chosen_ids_for_sim_;
  std::vector<int> fetched_plan_ids_;

  SIMULATE_TYPE sim_type_;
  MODE mode_;
};
}

#endif //FRAMEFAB_GUI_SELECTION_WIDGET_H
