#include <ros/console.h>
//#include "framefab_msgs/SurfaceBlendingParameters.h"

#include <framefab_gui/framefab_widget.h>
#include <framefab_gui/states/system_init_state.h>

#include <ui_framefab_widget.h>

//const std::string SURFACE_BLENDING_PARAMETERS_SERVICE = "surface_blending_parameters";
const static std::string SIMULATE_MOTION_PLAN_ACTION_SERVER_NAME = "simulate_motion_plan_as";
const static std::string MODEL_INPUT_GUI_ACTION_SERVER_NAME = "model_input_gui_as";

framefab_gui::FrameFabWidget::FrameFabWidget(QWidget* parent)
    : QWidget(parent),
      active_state_(NULL),
      input_ui_client_(MODEL_INPUT_GUI_ACTION_SERVER_NAME, true),
      simulate_motion_plan_action_client_(SIMULATE_MOTION_PLAN_ACTION_SERVER_NAME, true)
{
  // UI setup
  ui_ = new Ui::FrameFabWidget;
  ui_->setupUi(this);

  params_ = new ParamsSubmenu();
  params_->hide();

  // Starts in scan teach state
  changeState(new SystemInitState());

  // Wire in buttons
  connect(ui_->pushbutton_next, SIGNAL(clicked()), this, SLOT(onNextButton()));
  connect(ui_->pushbutton_back, SIGNAL(clicked()), this, SLOT(onBackButton()));
  connect(ui_->pushbutton_reset, SIGNAL(clicked()), this, SLOT(onResetButton()));
  connect(ui_->pushbutton_params, SIGNAL(clicked()), this, SLOT(onParamsButton()));

  // Connect to ROS services
  loadParameters();

  // Start Service Client
  ros::NodeHandle nh;
}

framefab_gui::FrameFabWidget::~FrameFabWidget()
{
  delete active_state_;
  delete params_;
}

void framefab_gui::FrameFabWidget::setText(const std::string& txt)
{
  ui_->textedit_status->setPlainText(QString::fromStdString(txt));
}

void framefab_gui::FrameFabWidget::appendText(const std::string& txt)
{
  ui_->textedit_status->moveCursor(QTextCursor::End);
  ui_->textedit_status->insertPlainText(QString::fromStdString(txt));
  ui_->textedit_status->moveCursor(QTextCursor::End);
}

void framefab_gui::FrameFabWidget::onNextButton()
{
  active_state_->onNext(*this);
}

void framefab_gui::FrameFabWidget::onBackButton()
{
  active_state_->onBack(*this);
}

void framefab_gui::FrameFabWidget::onResetButton()
{
  active_state_->onReset(*this);
}

void framefab_gui::FrameFabWidget::onParamsButton()
{
  params_->show();
}

//void framefab_gui::FrameFabWidget::onOptionsSave()
//{
//  ROS_INFO_STREAM("Save Options Called");
////  godel_msgs::SurfaceBlendingParameters msg;
////  msg.request.action = godel_msgs::SurfaceBlendingParameters::Request::SAVE_PARAMETERS;
////  msg.request.surface_detection = options_->surfaceDetectionParams();
////  msg.request.path_params = options_->pathPlanningParams();
////  msg.request.robot_scan = options_->robotScanParams();
////  msg.request.scan_plan = options_->scanParams();
////
////  if (!surface_blending_parameters_client_.call(msg.request, msg.response))
////    ROS_WARN_STREAM("Could not complete service call to save parameters!");
//}

void framefab_gui::FrameFabWidget::changeState(GuiState* new_state)
{
  // Don't transition to a null new state
  if (!new_state)
    return;

  if (active_state_)
  {
    active_state_->onExit(*this);
    delete active_state_;
  }

//  ptr_input_mainwindow_ = NULL;

//  real_client_ =
//      gui.nodeHandle().serviceClient<godel_msgs::SelectMotionPlan>(SELECT_MOTION_PLAN_SERVICE);

  active_state_ = new_state;
  connect(new_state, SIGNAL(newStateAvailable(GuiState*)), this, SLOT(changeState(GuiState*)));

  new_state->onStart(*this);
}

void framefab_gui::FrameFabWidget::setButtonsEnabled(bool enabled)
{
  ui_->pushbutton_next->setEnabled(enabled);
  ui_->pushbutton_back->setEnabled(enabled);
  ui_->pushbutton_reset->setEnabled(enabled);
  ui_->pushbutton_params->setEnabled(enabled);
}

void framefab_gui::FrameFabWidget::loadParameters()
{
//  framefab_msgs::SurfaceFrameFabParameters srv;
//  srv.request.action = srv.request.GET_CURRENT_PARAMETERS;
//  ros::ServiceClient param_client =
//      nodeHandle().serviceClient<framefab_msgs::SurfaceFrameFabParameters>(
//          "surface_framefab_parameters");
//
//  setButtonsEnabled(false);
//  param_client.waitForExistence();
//
//  if (param_client.call(srv))
//  {
//    this->options().setRobotScanParams(srv.response.robot_scan);
//    this->options().setSurfaceDetectionParams(srv.response.surface_detection);
//    this->options().setPathPlanningParams(srv.response.path_params);
//    this->options().setScanParams(srv.response.scan_plan);
//  }
//  else
//  {
//    ROS_WARN_STREAM("Unable to fetch framefab parameters");
//  }
//  setButtonsEnabled(true);
}

void framefab_gui::FrameFabWidget::setLabelText(const std::string& txt)
{
  ui_->label_status->setText( QString::fromStdString(txt));
}

void framefab_gui::FrameFabWidget::sendGoal(const bool enabled)
{
  framefab_msgs::ModelInputGuiActionGoal goal;
  goal.goal.enable_ui = enabled;

  input_ui_client_.sendGoal(goal.goal);
}

//void framefab_gui::FrameFabWidget::sendGoalAndWait(const framefab_msgs::ModelInputGuiActionGoal& goal)
//{
//  ros::Duration timeout = ros::Duration(60);
//  simulate_motion_plan_action_client_.sendGoalAndWait(goal.goal, timeout, timeout);
//}
