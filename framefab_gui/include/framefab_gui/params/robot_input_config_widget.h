//
// Created by yijiangh on 9/4/17.
//

#ifndef FRAMEFAB_GUI_ROBOT_INPUT_CONFIG_WIDGET_H
#define FRAMEFAB_GUI_ROBOT_INPUT_CONFIG_WIDGET_H

#include <framefab_gui/parameter_window_base.h>
#include <framefab_msgs/RobotInputParameters.h>

namespace Ui
{
class RobotInputConfigWindow;
}

namespace framefab_gui
{

class RobotInputConfigWidget : public ParameterWindowBase
{
  Q_OBJECT
 public:
  RobotInputConfigWidget(framefab_msgs::RobotInputParameters params);
  framefab_msgs::RobotInputParameters& params() { return params_; }

  virtual void update_display_fields();
  virtual void update_internal_fields();

 private:
  framefab_msgs::RobotInputParameters params_;
  Ui::RobotInputConfigWindow* ui_;
};
}

#endif //FRAMEFAB_GUI_ROBOT_INPUT_CONFIG_WIDGET_H
