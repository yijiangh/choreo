//
// Created by yijiangh on 9/9/17.
//

#ifndef CHOREO_GUI_OUTPUT_PATH_INPUT_CONFIG_WIDGET_H
#define CHOREO_GUI_OUTPUT_PATH_INPUT_CONFIG_WIDGET_H

#include <choreo_gui/parameter_window_base.h>
#include <choreo_msgs/OutputSaveDirInputParameters.h>

namespace Ui
{
class OutputSaveDirInputConfigWindow;
}

namespace choreo_gui
{

class OutputSaveDirInputConfigWidget : public ParameterWindowBase


{
  Q_OBJECT
 public:
  OutputSaveDirInputConfigWidget(choreo_msgs::OutputSaveDirInputParameters params);
  choreo_msgs::OutputSaveDirInputParameters& params() { return params_; }

  virtual void update_display_fields();
  virtual void update_internal_fields();

 protected Q_SLOTS:
  virtual void get_file_path_handler();

 private:
  choreo_msgs::OutputSaveDirInputParameters params_;
  Ui::OutputSaveDirInputConfigWindow* ui_;
  std::string last_filepath_;
};
}

#endif //CHOREO_GUI_OUTPUT_PATH_INPUT_CONFIG_WIDGET_H