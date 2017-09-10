#include <framefab_gui/params_submenu.h>
#include <ui_params_submenu.h>

framefab_gui::ParamsSubmenu::ParamsSubmenu(QWidget* parent) : QWidget(parent)
{
  ui_ = new Ui::ParamsSubmenu();
  ui_->setupUi(this);

  // Set up option menus
  // ModelInput
  model_input_widget_ = new ModelInputConfigWidget(framefab_msgs::ModelInputParameters());
  connect(ui_->pushbutton_model_input, SIGNAL(clicked()), model_input_widget_, SLOT(show()));

  // Path Input
  path_input_widget_ = new PathInputConfigWidget(framefab_msgs::PathInputParameters());
  connect(ui_->pushbutton_path_input, SIGNAL(clicked()), path_input_widget_, SLOT(show()));

  // Robot Input
  robot_input_widget_ = new RobotInputConfigWidget(framefab_msgs::RobotInputParameters());
  connect(ui_->pushbutton_robot_input, SIGNAL(clicked()), robot_input_widget_, SLOT(show()));

  // Output Path Input
  output_path_input_widget_ = new OutputPathInputConfigWidget(framefab_msgs::OutputPathInputParameters());
  connect(ui_->pushbutton_output_path_input, SIGNAL(clicked()), output_path_input_widget_, SLOT(show()));

  // Save Request Connection
  connect(model_input_widget_, SIGNAL(parameters_save_requested()), this, SIGNAL(saveRequested()));
  connect(path_input_widget_, SIGNAL(parameters_save_requested()), this, SIGNAL(saveRequested()));
  connect(robot_input_widget_, SIGNAL(parameters_save_requested()), this, SIGNAL(saveRequested()));
  connect(output_path_input_widget_, SIGNAL(parameters_save_requested()), this, SIGNAL(saveRequested()));

  // Accept Request Connection
  connect(model_input_widget_, SIGNAL(parameters_changed()), this, SIGNAL(acceptRequested()));
  connect(path_input_widget_, SIGNAL(parameters_changed()), this, SIGNAL(acceptRequested()));
  connect(robot_input_widget_, SIGNAL(parameters_changed()), this, SIGNAL(acceptRequested()));
  connect(output_path_input_widget_, SIGNAL(parameters_changed()), this, SIGNAL(acceptRequested()));
}

void framefab_gui::ParamsSubmenu::showOutputPathInputConfigWidget(bool enable)
{
  if(enable)
  {
    output_path_input_widget_->show();
  }
  else
  {
    output_path_input_widget_->hide();
  }
}

const framefab_msgs::ModelInputParameters& framefab_gui::ParamsSubmenu::modelInputParams() const
{
  return model_input_widget_->params();
}

void framefab_gui::ParamsSubmenu::setModelInputParams(
    const framefab_msgs::ModelInputParameters& params)
{
  model_input_widget_->params() = params;
  model_input_widget_->update_display_fields();
}

const framefab_msgs::PathInputParameters& framefab_gui::ParamsSubmenu::pathInputParams() const
{
  return path_input_widget_->params();
}

void framefab_gui::ParamsSubmenu::setPathInputParams(
    const framefab_msgs::PathInputParameters& params)
{
  path_input_widget_->params() = params;
  path_input_widget_->update_display_fields();
}

const framefab_msgs::RobotInputParameters& framefab_gui::ParamsSubmenu::robotInputParams() const
{
  return robot_input_widget_->params();
}

void framefab_gui::ParamsSubmenu::setRobotInputParams(const framefab_msgs::RobotInputParameters& params)
{
  robot_input_widget_->params() = params;
  robot_input_widget_->update_display_fields();
}

const framefab_msgs::OutputPathInputParameters& framefab_gui::ParamsSubmenu::outputPathInputParams() const
{
  return output_path_input_widget_->params();
}

void framefab_gui::ParamsSubmenu::setOutputPathInputParams(const framefab_msgs::OutputPathInputParameters& params)
{
  output_path_input_widget_->params() = params;
  output_path_input_widget_->update_display_fields();
}
