#include <ros/console.h>
#include "choreo_msgs/ChoreoParameters.h"

#include <choreo_gui/choreo_widget.h>
#include <choreo_gui/states/system_init_state.h>

#include <ui_choreo_widget.h>
#include <QtConcurrent/QtConcurrentRun>

// TODO temp
#include <actionlib/client/simple_action_client.h>
#include <choreo_msgs/ProcessPlanningAction.h>
const static std::string PROCESS_PLANNING_ACTION_CLIENT_NAME = "process_planning_action";

const std::string CHOREO_PARAMETERS_SERVICE = "choreo_parameters";
const static std::string SIMULATE_MOTION_PLAN_ACTION_SERVER_NAME = "simulate_motion_plan_as";

choreo_gui::ChoreoWidget::ChoreoWidget(QWidget* parent)
    : QWidget(parent),
      active_state_(NULL),
      simulate_motion_plan_action_client_(SIMULATE_MOTION_PLAN_ACTION_SERVER_NAME, true)
{
  // UI setup
  ui_ = new Ui::ChoreoWidget;
  ui_->setupUi(this);

//  ui_->tabWidget->setCurrentIndex(1);

  params_ = new ParamsSubmenu();
  params_->setWindowFlags(Qt::WindowStaysOnTopHint);

  selection_widget_ = new SelectionWidget();

  // Starts in scan teach state
  changeState(new SystemInitState());

  // Wire in buttons
  connect(ui_->pushbutton_next, SIGNAL(clicked()), this, SLOT(onNextButton()));
  connect(ui_->pushbutton_back, SIGNAL(clicked()), this, SLOT(onBackButton()));
  connect(ui_->pushbutton_reset, SIGNAL(clicked()), this, SLOT(onResetButton()));
  connect(ui_->pushbutton_params, SIGNAL(clicked()), this, SLOT(onParamsButton()));
  connect(ui_->pushbutton_picknplace, SIGNAL(clicked()), this, SLOT(onPicknPlace()));

  // wire in picknplace button

  // Wire in params signals
  connect(params_, SIGNAL(saveRequested()), this, SLOT(onParamsSave()));
  connect(params_, SIGNAL(acceptRequested()), this, SLOT(onParamsAccept()));

  // Wire in selection signals
  connect(selection_widget_, SIGNAL(enterSelectionWidget()), this, SLOT(onDisableButtons()));
  connect(selection_widget_, SIGNAL(exitSelectionWidget()), this, SLOT(onEnableButtons()));
  connect(selection_widget_, SIGNAL(enableChoreoWidgetButtons()), this, SLOT(onEnableButtons()));
  connect(selection_widget_, SIGNAL(setOutputSaveDirOn()), this, SLOT(showOutputSaveDirParams()));

  // Connect to ROS save params services
  loadParameters();

  // Start Service Client
  ros::NodeHandle nh;
  choreo_parameters_client_ =
      nh.serviceClient<choreo_msgs::ChoreoParameters>(CHOREO_PARAMETERS_SERVICE);
}

choreo_gui::ChoreoWidget::~ChoreoWidget()
{
  delete active_state_;
  delete params_;
}

void choreo_gui::ChoreoWidget::setText(const std::string& txt)
{
  ui_->textedit_status->setPlainText(QString::fromStdString(txt));
}

void choreo_gui::ChoreoWidget::appendText(const std::string& txt)
{
  ui_->textedit_status->moveCursor(QTextCursor::End);
  ui_->textedit_status->insertPlainText(QString::fromStdString(txt));
  ui_->textedit_status->moveCursor(QTextCursor::End);
}

void choreo_gui::ChoreoWidget::onNextButton()
{
  active_state_->onNext(*this);
}

void choreo_gui::ChoreoWidget::onBackButton()
{
  active_state_->onBack(*this);
}

void choreo_gui::ChoreoWidget::onResetButton()
{
  active_state_->onReset(*this);

  choreo_msgs::SimulateMotionPlanGoal goal;
  goal.action = choreo_msgs::SimulateMotionPlanGoal::RESET_TO_DEFAULT_POSE;
  goal.simulate = true;
  goal.wait_for_execution = true;

  QtConcurrent::run(this, &ChoreoWidget::sendGoalAndWait, goal);
}

void choreo_gui::ChoreoWidget::onParamsButton()
{
  params_->show();

}

void choreo_gui::ChoreoWidget::onParamsSave()
{
  choreo_msgs::ChoreoParameters msg;
  msg.request.action = choreo_msgs::ChoreoParameters::Request::SAVE_PARAMETERS;
  msg.request.model_params = params_->modelInputParams();
  msg.request.task_sequence_params = params_->taskSequenceInputParams();
  msg.request.robot_params = params_->robotInputParams();
  msg.request.output_save_dir_params = params_->outputSaveDirInputParams();

  if (!choreo_parameters_client_.call(msg.request, msg.response))
    ROS_WARN_STREAM("Could not complete service call to save parameters!");
}

void choreo_gui::ChoreoWidget::onParamsAccept()
{
  choreo_msgs::ChoreoParameters msg;
  msg.request.action = choreo_msgs::ChoreoParameters::Request::SET_PARAMETERS;
  msg.request.model_params = params_->modelInputParams();
  msg.request.task_sequence_params = params_->taskSequenceInputParams();
  msg.request.robot_params = params_->robotInputParams();
  msg.request.output_save_dir_params = params_->outputSaveDirInputParams();

  if (!choreo_parameters_client_.call(msg.request, msg.response))
    ROS_WARN_STREAM("Could not complete service call to set parameters!");
}

void choreo_gui::ChoreoWidget::onEnableButtons()
{
  setButtonsEnabled(true);
}

void choreo_gui::ChoreoWidget::onDisableButtons()
{
  setButtonsEnabled(false);
}

void choreo_gui::ChoreoWidget::onPicknPlace()
{
  choreo_msgs::ProcessPlanningGoal goal;
  goal.action = choreo_msgs::ProcessPlanningGoal::PICKNPLACE_TEST;

  ROS_INFO("Waiting for pickn place process planning action server to start.");
  actionlib::SimpleActionClient<choreo_msgs::ProcessPlanningAction>
      process_planning_action_client(PROCESS_PLANNING_ACTION_CLIENT_NAME, true);
  process_planning_action_client.waitForServer();

  if(process_planning_action_client.isServerConnected())
  {
    ROS_INFO_STREAM("process planning action server connected!");
  }
  else
  {
    ROS_WARN_STREAM("action process planning server not connected");
  }

  process_planning_action_client.sendGoal(goal);
  ROS_INFO_STREAM("Goal sent from pick n place process planning state");
}

void choreo_gui::ChoreoWidget::changeState(GuiState* new_state)
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

void choreo_gui::ChoreoWidget::showOutputSaveDirParams()
{
  this->params().showOutputSaveDirInputConfigWidget(true);
}

void choreo_gui::ChoreoWidget::setButtonsEnabled(bool enabled)
{
  ui_->pushbutton_next->setEnabled(enabled);
  ui_->pushbutton_back->setEnabled(enabled);
  ui_->pushbutton_reset->setEnabled(enabled);
}

void choreo_gui::ChoreoWidget::setParamsButtonEnabled(bool enabled)
{
  ui_->pushbutton_params->setEnabled(enabled);
}

void choreo_gui::ChoreoWidget::loadParameters()
{
  choreo_msgs::ChoreoParameters srv;
  srv.request.action = srv.request.GET_CURRENT_PARAMETERS;
  ros::ServiceClient param_client =
      nodeHandle().serviceClient<choreo_msgs::ChoreoParameters>(CHOREO_PARAMETERS_SERVICE);

  setButtonsEnabled(false);
  if(!param_client.waitForExistence(ros::Duration(10)))
  {
    ROS_ERROR("[UI] Unable to connect to parameter server in core service!");
  }
  else
  {
    ROS_INFO_STREAM("[UI] Connected to parameter server in core service.");
  }

  if (param_client.call(srv))
  {
    this->params().setModelInputParams(srv.response.model_params);
    this->params().setTaskSequenceInputParams(srv.response.task_sequence_params);
    this->params().setRobotInputParams(srv.response.robot_params);
    this->params().setOutputSaveDirInputParams(srv.response.output_save_dir_params);
  }
  else
  {
    ROS_WARN_STREAM("[UI] Unable to fetch choreo parameters");
  }
  setButtonsEnabled(true);
}

void choreo_gui::ChoreoWidget::setLabelText(const std::string& txt)
{
  ui_->label_status->setText( QString::fromStdString(txt));
}

void choreo_gui::ChoreoWidget::sendGoal(const choreo_msgs::SimulateMotionPlanGoal& goal)
{
  simulate_motion_plan_action_client_.sendGoal(goal);
}

void choreo_gui::ChoreoWidget::sendGoalAndWait(const choreo_msgs::SimulateMotionPlanGoal& goal)
{
  ros::Duration timeout = ros::Duration(60);
  simulate_motion_plan_action_client_.sendGoalAndWait(goal, timeout, timeout);
}