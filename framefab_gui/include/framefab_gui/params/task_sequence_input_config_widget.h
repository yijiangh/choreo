//
// Created by yijiangh on 6/17/17.
//

#ifndef FRAMEFAB_GUI_TEST_PROCESS_CONFIG_WIDGET_H
#define FRAMEFAB_GUI_TEST_PROCESS_CONFIG_WIDGET_H

#include <framefab_gui/parameter_window_base.h>
#include <framefab_msgs/TaskSequenceInputParameters.h>

namespace Ui
{
class TaskSequenceInputConfigWindow;
}

namespace framefab_gui
{

class TaskSequenceInputConfigWidget : public ParameterWindowBase
{
  Q_OBJECT
 public:
  TaskSequenceInputConfigWidget(framefab_msgs::TaskSequenceInputParameters params);
  framefab_msgs::TaskSequenceInputParameters& params() { return params_; }

  virtual void update_display_fields();
  virtual void update_internal_fields();

 protected Q_SLOTS:
  virtual void get_file_path_handler();

 private:
  framefab_msgs::TaskSequenceInputParameters params_;
  Ui::TaskSequenceInputConfigWindow* ui_;
  std::string last_filepath_;
};
}

#endif //FRAMEFAB_GUI_TEST_PROCESS_CONFIG_WIDGET_H
