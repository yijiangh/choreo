#include <ros/console.h>
#include "framefab_msgs/FrameFabParameters.h"

#include <framefab_gui/framefab_widget.h>
#include <framefab_gui/states/system_init_state.h>

#include <ui_framefab_widget.h>

const std::string FRAMEFAB_PARAMETERS_SERVICE = "framefab_parameters";
const static std::string SIMULATE_MOTION_PLAN_ACTION_SERVER_NAME = "simulate_motion_plan_as";

framefab_gui::FrameFabWidget::FrameFabWidget(QWidget* parent)
    : QWidget(parent),
      active_state_(NULL),
      simulate_motion_plan_action_client_(SIMULATE_MOTION_PLAN_ACTION_SERVER_NAME, true)
{
  // UI setup
  ui_ = new Ui::FrameFabWidget;
  ui_->setupUi(this);

  params_ = new ParamsSubmenu();

  select_path_ = new SelectPathWidget();

  // Starts in scan teach state
  changeState(new SystemInitState());

  // Wire in buttons
  connect(ui_->pushbutton_next, SIGNAL(clicked()), this, SLOT(onNextButton()));
  connect(ui_->pushbutton_back, SIGNAL(clicked()), this, SLOT(onBackButton()));
  connect(ui_->pushbutton_reset, SIGNAL(clicked()), this, SLOT(onResetButton()));
  connect(ui_->pushbutton_params, SIGNAL(clicked()), this, SLOT(onParamsButton()));

  // Wire in params signals
  connect(params_, SIGNAL(saveRequested()), this, SLOT(onParamsSave()));
  connect(params_, SIGNAL(acceptRequested()), this, SLOT(onParamsAccept()));

  // Wire in selection signals
  connect(select_path_, SIGNAL(acceptSelection()), this, SLOT(onNextButton()));

  // Connect to ROS save params services
  loadParameters();

  // Start Service Client
  ros::NodeHandle nh;
  framefab_parameters_client_ =
      nh.serviceClient<framefab_msgs::FrameFabParameters>(FRAMEFAB_PARAMETERS_SERVICE);
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

void framefab_gui::FrameFabWidget::onParamsSave()
{
  framefab_msgs::FrameFabParameters msg;
  msg.request.action = framefab_msgs::FrameFabParameters::Request::SAVE_PARAMETERS;
  msg.request.model_params = params_->modelInputParams();
  msg.request.path_params = params_->pathInputParams();

  if (!framefab_parameters_client_.call(msg.request, msg.response))
    ROS_WARN_STREAM("Could not complete service call to save parameters!");
}

void framefab_gui::FrameFabWidget::onParamsAccept()
{
  framefab_msgs::FrameFabParameters msg;
  msg.request.action = framefab_msgs::FrameFabParameters::Request::SET_PARAMETERS;
  msg.request.model_params = params_->modelInputParams();
  msg.request.path_params = params_->pathInputParams();

  if (!framefab_parameters_client_.call(msg.request, msg.response))
    ROS_WARN_STREAM("Could not complete service call to set parameters!");
}

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
  ui_->pushbutton_next->setEnabled(enabled);
  ui_->pushbutton_back->setEnabled(enabled);
  ui_->pushbutton_reset->setEnabled(enabled);
}

void framefab_gui::FrameFabWidget::setParamsButtonEnabled(bool enabled)
{
  ui_->pushbutton_params->setEnabled(enabled);
}

void framefab_gui::FrameFabWidget::loadParameters()
{
  framefab_msgs::FrameFabParameters srv;
  srv.request.action = srv.request.GET_CURRENT_PARAMETERS;
  ros::ServiceClient param_client =
      nodeHandle().serviceClient<framefab_msgs::FrameFabParameters>(FRAMEFAB_PARAMETERS_SERVICE);

  setButtonsEnabled(false);
  param_client.waitForExistence();

  if (param_client.call(srv))
  {
    this->params().setModelInputParams(srv.response.model_params);
    this->params().setPathInputParams(srv.response.path_params);
  }
  else
  {
    ROS_WARN_STREAM("Unable to fetch framefab parameters");
  }
  setButtonsEnabled(true);
}

void framefab_gui::FrameFabWidget::setLabelText(const std::string& txt)
{
  ui_->label_status->setText( QString::fromStdString(txt));
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