//
// Created by yijiangh on 6/17/17.
//

#ifndef FRAMEFAB_GUI_TEST_PROCESS_CONFIG_WIDGET_H
#define FRAMEFAB_GUI_TEST_PROCESS_CONFIG_WIDGET_H

#include <framefab_gui/parameter_window_base.h>

#include <framefab_msgs/ModelInputParameters.h>

namespace Ui
{
class ModelInputConfigWindow;
}

namespace framefab_gui
{

class ModelInputConfigWidget : public ParameterWindowBase
{
  Q_OBJECT
 public:
  ModelInputConfigWidget(framefab_msgs::ModelInputParameters params);
  framefab_msgs::ModelInputParameters& params() { return params_; }

  virtual void update_display_fields();
  virtual void update_internal_fields();

 protected:
  // Maps scan-method enum to a field inside the combo-box display
  virtual int get_unit_combobox_value();

//  // Maps a field in combo-box display to scan-method enum
//  virtual int get_scan_method_enum_value();

 protected Q_SLOTS:
  virtual void get_file_path_handler();

 private:
  framefab_msgs::ModelInputParameters params_;
  Ui::ModelInputConfigWindow* ui_;
};
}

#endif //FRAMEFAB_GUI_TEST_PROCESS_CONFIG_WIDGET_H
