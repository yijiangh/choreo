//
// Created by yijiangh on 9/9/17.
//

#ifndef FRAMEFAB_GUI_OUTPUT_PATH_INPUT_CONFIG_WIDGET_H
#define FRAMEFAB_GUI_OUTPUT_PATH_INPUT_CONFIG_WIDGET_H

#include <framefab_gui/parameter_window_base.h>
#include <framefab_msgs/OutputPathInputParameters.h>

namespace Ui
{
class OutputPathInputConfigWindow;
}

namespace framefab_gui
{

class OutputPathInputConfigWidget : public ParameterWindowBase
{
  Q_OBJECT
 public:
  OutputPathInputConfigWidget(framefab_msgs::OutputPathInputParameters params);
  framefab_msgs::OutputPathInputParameters& params() { return params_; }

  virtual void update_display_fields();
  virtual void update_internal_fields();

 protected Q_SLOTS:
  virtual void get_file_path_handler();

 private:
  framefab_msgs::OutputPathInputParameters params_;
  Ui::OutputPathInputConfigWindow* ui_;
  std::string last_filepath_;
};
}

#endif //FRAMEFAB_GUI_OUTPUT_PATH_INPUT_CONFIG_WIDGET_H