#include <ros/console.h>

//#include "framefab_msgs/SurfaceBlendingParameters.h"

#include <framefab_gui/framefab_widget.h>
#include <framefab_gui/states/system_init_state.h>

#include <ui_framefab_widget.h>

//const std::string SURFACE_BLENDING_PARAMETERS_SERVICE = "surface_blending_parameters";
const static std::string SIMULATE_MOTION_PLAN_ACTION_SERVER_NAME = "simulate_motion_plan_as";

framefab_gui::FrameFabWidget::FrameFabWidget(QWidget* parent)
    : QWidget(parent),
      active_state_(NULL),
      simulate_motion_plan_action_client_(SIMULATE_MOTION_PLAN_ACTION_SERVER_NAME, true)
{
  // UI setup
  ui_ = new Ui::FrameFabWidget;
  ui_->setupUi(this);

//  params_ = new ParamsSubmenu();

  // Starts in scan teach state
  changeState(new SystemInitState());

  // Wire in buttons
  connect(ui_->pushButtonNext, SIGNAL(clicked()), this, SLOT(onNextButton()));
  connect(ui_->pushButtonBack, SIGNAL(clicked()), this, SLOT(onBackButton()));
  connect(ui_->pushButtonReset, SIGNAL(clicked()), this, SLOT(onResetButton()));

  // Connect to ROS services
  loadParameters();

  // Start Service Client
  ros::NodeHandle nh;

//  surface_blending_parameters_client_ =
//      nh.serviceClient<framefab_msgs::SurfaceBlendingParameters>(SURFACE_BLENDING_PARAMETERS_SERVICE);
}

framefab_gui::FrameFabWidget::~FrameFabWidget()
{
  delete active_state_;
}

void framefab_gui::FrameFabWidget::setText(const std::string& txt)
{
  ui_->textEditStatus->setPlainText(QString::fromStdString(txt));
}

void framefab_gui::FrameFabWidget::appendText(const std::string& txt)
{
  ui_->textEditStatus->moveCursor(QTextCursor::End);
  ui_->textEditStatus->insertPlainText(QString::fromStdString(txt));
  ui_->textEditStatus->moveCursor(QTextCursor::End);
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

  active_state_ = new_state;
  connect(new_state, SIGNAL(newStateAvailable(GuiState*)), this, SLOT(changeState(GuiState*)));

  new_state->onStart(*this);
}

void framefab_gui::FrameFabWidget::setButtonsEnabled(bool enabled)
{
  ui_->pushButtonNext->setEnabled(enabled);
  ui_->pushButtonBack->setEnabled(enabled);
  ui_->pushButtonReset->setEnabled(enabled);
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
  ui_->statusLabel->setText( QString::fromStdString(txt));
}

void framefab_gui::FrameFabWidget::sendGoal(const framefab_msgs::SimulateMotionPlanActionGoal& goal)
{
  simulate_motion_plan_action_client_.sendGoal(goal.goal);
}

void framefab_gui::FrameFabWidget::sendGoalAndWait(const framefab_msgs::SimulateMotionPlanActionGoal& goal)
{
  ros::Duration timeout = ros::Duration(60);
  simulate_motion_plan_action_client_.sendGoalAndWait(goal.goal, timeout, timeout);
}