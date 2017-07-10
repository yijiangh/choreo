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

  int getSelectedValue() { return print_order_; }

  // send srv to clean up visualization markers
  void cleanUpVisual();

  Q_SIGNALS:
  void acceptSelection();

 protected Q_SLOTS:
  // different source that changes order_value
  void buttonForwardUpdateOrderValue();
  void buttonBackwardUpdateOrderValue();
  void buttonSelectAll();
  void sliderUpdateOrderValue(int value);
  void lineeditUpdateOrderValue();

 private:
  ros::NodeHandle nh_;

  ros::ServiceClient visualize_client_;

  Ui::SelectionWidgetWindow* ui_;

  int max_value_;
  int print_order_;

  MODE mode_;
};
}

#endif //FRAMEFAB_GUI_SELECTION_WIDGET_H
