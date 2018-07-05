//
// Created by yijiangh on 6/17/17.
//

#ifndef CHOREO_RVIZ_PLUGIN_MODEL_INPUT_CONFIG_WIDGET_H
#define CHOREO_RVIZ_PLUGIN_MODEL_INPUT_CONFIG_WIDGET_H

#include "choreo_gui/parameter_window_base.h"

#include <choreo_msgs/ModelInputParameters.h>

namespace Ui
{
class ModelInputConfigWindow;
}

namespace choreo_gui
{

class ModelInputConfigWidget : public ParameterWindowBase
{
  Q_OBJECT
 public:
  ModelInputConfigWidget(choreo_msgs::ModelInputParameters params);
  choreo_msgs::ModelInputParameters& params() { return params_; }

  virtual void update_display_fields();
  virtual void update_internal_fields();

 protected:
  virtual int get_unit_combobox_value();

 protected Q_SLOTS:
  virtual void get_file_path_handler();

 private:
  choreo_msgs::ModelInputParameters params_;
  Ui::ModelInputConfigWindow* ui_;
  std::string last_filepath_;
};
}

#endif //CHOREO_RVIZ_PLUGIN_MODEL_INPUT_CONFIG_WIDGET_H