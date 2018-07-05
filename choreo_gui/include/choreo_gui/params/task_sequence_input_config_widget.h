//
// Created by yijiangh on 6/17/17.
//

#ifndef CHOREO_GUI_TEST_PROCESS_CONFIG_WIDGET_H
#define CHOREO_GUI_TEST_PROCESS_CONFIG_WIDGET_H

#include <choreo_gui/parameter_window_base.h>
#include <choreo_msgs/TaskSequenceInputParameters.h>

namespace Ui
{
class TaskSequenceInputConfigWindow;
}

namespace choreo_gui
{

class TaskSequenceInputConfigWidget : public ParameterWindowBase
{
  Q_OBJECT
 public:
  TaskSequenceInputConfigWidget(choreo_msgs::TaskSequenceInputParameters params);
  choreo_msgs::TaskSequenceInputParameters& params() { return params_; }

  virtual void update_display_fields();
  virtual void update_internal_fields();

 protected Q_SLOTS:
  virtual void get_file_path_handler();

 private:
  choreo_msgs::TaskSequenceInputParameters params_;
  Ui::TaskSequenceInputConfigWindow* ui_;
  std::string last_filepath_;
};
}

#endif //CHOREO_GUI_TEST_PROCESS_CONFIG_WIDGET_H
