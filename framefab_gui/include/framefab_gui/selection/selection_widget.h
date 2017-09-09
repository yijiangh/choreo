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

  int  getSelectedValue() { return print_order_; }
  bool getSimulateType() { return simulate_single_; }

  // send srv to clean up visualization markers
  void cleanUpVisual();

 protected:
  virtual void showEvent(QShowEvent *ev);
  virtual void closeEvent(QCloseEvent *ev);

 Q_SIGNALS:
  void acceptSelection();
  void simulateTypeChange(bool type);
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
  void buttonSimulate(bool single);
  void buttonCloseWidget();
  void buttonAcceptSelection();
  void sliderUpdateOrderValue(int value);
  void lineeditUpdateOrderValue();

  void widgetStateChanged();

 private:
  ros::NodeHandle nh_;

  ros::ServiceClient visualize_client_;

  Ui::SelectionWidgetWindow* ui_;

  int max_value_;
  int print_order_;

  bool simulate_single_;

  MODE mode_;
};
}

#endif //FRAMEFAB_GUI_SELECTION_WIDGET_H
