//
// Created by yijiangh on 9/4/17.
//

#ifndef CHOREO_GUI_ROBOT_INPUT_CONFIG_WIDGET_H
#define CHOREO_GUI_ROBOT_INPUT_CONFIG_WIDGET_H

#include <choreo_gui/parameter_window_base.h>
#include <choreo_msgs/RobotInputParameters.h>

namespace Ui
{
class RobotInputConfigWindow;
}

namespace choreo_gui
{

class RobotInputConfigWidget : public ParameterWindowBase
{
  Q_OBJECT
 public:
  RobotInputConfigWidget(choreo_msgs::RobotInputParameters params);
  choreo_msgs::RobotInputParameters& params() { return params_; }

  virtual void update_display_fields();
  virtual void update_internal_fields();

 private:
  choreo_msgs::RobotInputParameters params_;
  Ui::RobotInputConfigWindow* ui_;
};
}

#endif //CHOREO_GUI_ROBOT_INPUT_CONFIG_WIDGET_H
