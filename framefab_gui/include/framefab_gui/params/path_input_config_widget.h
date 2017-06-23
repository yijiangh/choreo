//
// Created by yijiangh on 6/17/17.
//

#ifndef FRAMEFAB_GUI_TEST_PROCESS_CONFIG_WIDGET_H
#define FRAMEFAB_GUI_TEST_PROCESS_CONFIG_WIDGET_H

#include <framefab_gui/parameter_window_base.h>
#include <framefab_msgs/PathInputParameters.h>

namespace Ui
{
class PathInputConfigWindow;
}

namespace framefab_gui
{

class PathInputConfigWidget : public ParameterWindowBase
{
  Q_OBJECT
 public:
  PathInputConfigWidget(framefab_msgs::PathInputParameters params);
  framefab_msgs::PathInputParameters& params() { return params_; }

  virtual void update_display_fields();
  virtual void update_internal_fields();

 protected Q_SLOTS:
  virtual void get_file_path_handler();

 private:
  framefab_msgs::PathInputParameters params_;
  Ui::PathInputConfigWindow* ui_;
};
}

#endif //FRAMEFAB_GUI_TEST_PROCESS_CONFIG_WIDGET_H
