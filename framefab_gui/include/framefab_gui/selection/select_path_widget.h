//
// Created by yijiangh on 6/27/17.
//

#ifndef FRAMEFAB_GUI_SELECT_PATH_WIDGET_H
#define FRAMEFAB_GUI_SELECT_PATH_WIDGET_H

#include <QString>
#include <QWidget>

#include <ros/ros.h>

namespace Ui
{
class SelectPathWidgetWindow;
}

namespace framefab_gui
{

class SelectPathWidget : public QWidget
{
  Q_OBJECT
 public:
  SelectPathWidget(QWidget* parent = 0);

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
  void sliderUpdateOrderValue(int value);
  void lineeditUpdateOrderValue();

 private:
  ros::NodeHandle nh_;

  ros::ServiceClient visualize_client_;

  Ui::SelectPathWidgetWindow* ui_;

  int max_value_;
  int print_order_;
};
}

#endif //FRAMEFAB_GUI_SELECT_PATH_WIDGET_H
