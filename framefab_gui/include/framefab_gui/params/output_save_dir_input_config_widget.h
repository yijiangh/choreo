//
// Created by yijiangh on 9/9/17.
//

#ifndef FRAMEFAB_GUI_OUTPUT_PATH_INPUT_CONFIG_WIDGET_H
#define FRAMEFAB_GUI_OUTPUT_PATH_INPUT_CONFIG_WIDGET_H

#include <framefab_gui/parameter_window_base.h>
#include <framefab_msgs/OutputSaveDirInputParameters.h>

namespace Ui
{
class OutputSaveDirInputConfigWindow;
}

namespace framefab_gui
{

class OutputSaveDirInputConfigWidget : public ParameterWindowBase


{
  Q_OBJECT
 public:
  OutputSaveDirInputConfigWidget(framefab_msgs::OutputSaveDirInputParameters params);
  framefab_msgs::OutputSaveDirInputParameters& params() { return params_; }

  virtual void update_display_fields();
  virtual void update_internal_fields();

 protected Q_SLOTS:
  virtual void get_file_path_handler();

 private:
  framefab_msgs::OutputSaveDirInputParameters params_;
  Ui::OutputSaveDirInputConfigWindow* ui_;
  std::string last_filepath_;
};
}

#endif //FRAMEFAB_GUI_OUTPUT_PATH_INPUT_CONFIG_WIDGET_H