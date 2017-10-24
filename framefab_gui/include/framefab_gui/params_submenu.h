#ifndef PARAMS_SUBMENU_H
#define PARAMS_SUBMENU_H

#include <QWidget>

#include "framefab_gui/params/model_input_config_widget.h"
#include "framefab_gui/params/task_sequence_input_config_widget.h"
#include "framefab_gui/params/robot_input_config_widget.h"
#include "framefab_gui/params/output_save_dir_input_config_widget.h"

namespace Ui
{
class ParamsSubmenu;
}

namespace framefab_gui
{

class ParamsSubmenu : public QWidget
{
  Q_OBJECT

 public:
  ParamsSubmenu(QWidget* parent = 0);

  void showOutputSaveDirInputConfigWidget(bool enable);

  // For every submenu / set of parameters, we have getters/setters
  const framefab_msgs::ModelInputParameters& modelInputParams() const;
  void  setModelInputParams(const framefab_msgs::ModelInputParameters& params);

  const framefab_msgs::TaskSequenceInputParameters& taskSequenceInputParams() const;
  void setTaskSequenceInputParams(const framefab_msgs::TaskSequenceInputParameters& params);

  const framefab_msgs::RobotInputParameters& robotInputParams() const;
  void setRobotInputParams(const framefab_msgs::RobotInputParameters& params);

  const framefab_msgs::OutputSaveDirInputParameters& outputSaveDirInputParams() const;
  void setOutputSaveDirInputParams(const framefab_msgs::OutputSaveDirInputParameters& params);

  Q_SIGNALS:
  void saveRequested();
  void acceptRequested();

 private:
  // Display layout
  Ui::ParamsSubmenu* ui_;

  // Configuration components
  ModelInputConfigWidget*           model_input_widget_;
  TaskSequenceInputConfigWidget*    task_sequence_input_widget_;
  RobotInputConfigWidget*           robot_input_widget_;
  OutputSaveDirInputConfigWidget*   output_save_dir_input_widget_;
};
} // end namespace framefab_gui

#endif // PARAMS_SUBMENU_H