//
// Created by yijiangh on 6/27/17.
//

#ifndef FRAMEFAB_GUI_SELECT_PATH_WIDGET_H
#define FRAMEFAB_GUI_SELECT_PATH_WIDGET_H

#include <QString>
#include <QWidget>

#include <ros/ros.h>
#include <frmaefab_gui/framefab_widget.h>

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
  SelectPathWidget(FrameFabWidget& gui);

  // service request on required parameters
  void loadParameters();

  // set max value, update slider & lineedit
  void setMaxValue(int m);

  // update display according to current print order
  void orderValueChanged();

  void setInputEnabled(bool enabled);

 protected Q_SLOTS:
  void acceptButtonHandler();

  // different source that changes order_value
  void buttonForwardUpdateOrderValue();
  void buttonBackwardUpdateOrderValue();
  void sliderUpdateOrderValue(int value);
  void lineeditUpdateOrderValue(QString value);

 private:
  ros::NodeHandle nh_;

  ros::ServiceClient visualize_client_;

  Ui::SelectPathWidgetWindow* ui_;

  FrameFabWidget* ptr_gui_;

  int max_value_;
  int print_order_;
};
}

#endif //FRAMEFAB_GUI_SELECT_PATH_WIDGET_H
