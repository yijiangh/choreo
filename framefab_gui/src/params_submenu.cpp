#include <framefab_gui/params_submenu.h>

#include <ui_params_submenu.h>

//#include "framefab_gui/options/robot_scan_configuration.h"

framefab_gui::ParamsSubmenu::ParamsSubmenu(QWidget* parent) : QWidget(parent)
{
  ui_ = new Ui::ParamsSubmenu();
  ui_->setupUi(this);
  // Set up option menus
  //// Robot Scan
  model_input_widget_ = new ModelInputConfigWidget(framefab_msgs::ModelInputParameters());
  connect(ui_->pushbutton_model_input, SIGNAL(clicked()), model_input_widget_, SLOT(show()));
  //// Surface Detection
//  test_process_widget_ = new TestProcessConfigWidget(framefab_msgs::TestProcessParameters());
//  connect(ui_->pushbutton_test_process, SIGNAL(clicked()), test_process_widget_, SLOT(show()));
  // Path Planning
//  path_planning_params_ = new PathPlanningConfigWidget(framefab_msgs::PathPlanningParameters());
//  connect(ui_->pushButtonPathPlanningParams, SIGNAL(clicked()), path_planning_params_, SLOT(show()));
//  //// Scan (QA) params
//  scan_params_ = new ScanPlanConfigWidget(framefab_msgs::ScanPlanParameters());
//  connect(ui_->pushButtonQAParams, SIGNAL(clicked()), scan_params_, SLOT(show()));
//
  connect(model_input_widget_, SIGNAL(parameters_save_requested()), this, SIGNAL(saveRequested()));
  //connect(test_process_widget_, SIGNAL(parameters_save_requested()), this, SIGNAL(saveRequested()));
//  connect(path_planning_params_, SIGNAL(parameters_save_requested()), this, SIGNAL(saveRequested()));
//  connect(scan_params_, SIGNAL(parameters_save_requested()), this, SIGNAL(saveRequested()));
}

const framefab_msgs::ModelInputParameters& framefab_gui::ParamsSubmenu::ModelInputParams() const
{
  return model_input_widget_->params();
}

void framefab_gui::ParamsSubmenu::setModelInputParams(
    const framefab_msgs::ModelInputParameters& params)
{
  model_input_widget_->params() = params;
  model_input_widget_->update_display_fields();
}

//const framefab_msgs::TestProcessParameters&
//framefab_gui::ParamsSubmenu::TestProcessParams() const
//{
//  return test_process_widget_->params();
//}
//
//void framefab_gui::ParamsSubmenu::setTestProcessParams(
//    const framefab_msgs::TestProcessParameters& params)
//{
////  test_process_->params() = params;
////  test_process_->update_display_fields();
//}

//const framefab_msgs::PathPlanningParameters& framefab_gui::ParamsSubmenu::pathPlanningParams() const
//{
//  return path_planning_params_->params();
//}

//void framefab_gui::ParamsSubmenu::setPathPlanningParams(const framefab_msgs::PathPlanningParameters& params)
//{
//  path_planning_params_->params() = params;
//  path_planning_params_->update_display_fields();
//}

//const framefab_msgs::ScanPlanParameters& framefab_gui::ParamsSubmenu::scanParams() const
//{
//  return scan_params_->params();
//}
//
//void framefab_gui::ParamsSubmenu::setScanParams(const framefab_msgs::ScanPlanParameters& params)
//{
//  scan_params_->params() = params;
//  scan_params_->update_display_fields();
//}