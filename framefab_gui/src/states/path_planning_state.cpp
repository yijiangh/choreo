//#include "framefab_gui/states/scanning_state.h"

#include <ros/console.h>
#include <framefab_gui/framefab_widget.h>
#include <framefab_gui/states/system_init_state.h>
#include <framefab_gui/states/path_planning_state.h>
#include <QtConcurrent/QtConcurrentRun>

// input params
#include <framefab_msgs/PathPlanning.h>
#include <framefab_msgs/ModelInputParameters.h>
#include <framefab_msgs/PathInputParameters.h>

framefab_gui::PathPlanningState::PathPlanningState()
    : path_planning_action_client_(PATH_PLANNING_ACTION_SERVER_NAME, true),
      f_ac_("f", true)
{
//  ROS_INFO_STREAM("PathPlanningState Init.");
}

framefab_gui::PathPlanningState::~PathPlanningState()
{}

void framefab_gui::PathPlanningState::onStart(FrameFabWidget& gui)
{
  gui.setText("PathPlanning State.\nPlease input data in parameter widget.\nClick 'Next' to continue after finished.");
//  gui.setButtonsEnabled(false);

  gui_ptr_ = &gui;

  QObject::connect(this, SIGNAL(feedbackReceived(QString)), this, SLOT(setFeedbackText(QString)));
  QtConcurrent::run(this, &PathPlanningState::makeRequest, gui.params().modelInputParams(),
                    gui.params().pathInputParams());
}

void framefab_gui::PathPlanningState::onExit(FrameFabWidget& gui) { gui.setButtonsEnabled(true); }

void framefab_gui::PathPlanningState::onNext(FrameFabWidget& gui)
{
//  Q_EMIT newStateAvailable(new SelectPathState());
}

void framefab_gui::PathPlanningState::onBack(FrameFabWidget& gui) {}

void framefab_gui::PathPlanningState::onReset(FrameFabWidget& gui) {}

void framefab_gui::PathPlanningState::makeRequest(
    framefab_msgs::ModelInputParameters model_params,
    framefab_msgs::PathInputParameters path_params)
{
//  framefab_msgs::PathPlanningGoal goal;
//  goal.action = framefab_msgs::PathPlanningGoal::FIND_AND_PROCESS;
//  goal.use_default_parameters = false;
//  goal.model_params = model_params;
//  goal.path_params  = path_params;
//
//  if(path_planning_action_client_.isServerConnected())
//{ ROS_INFO_STREAM("action server connected!");}
//else
//{ ROS_WARN_STREAM("action server not connected");}
//
//  path_planning_action_client_.sendGoal(
//      goal,
//      boost::bind(&framefab_gui::PathPlanningState::pathPlanningDoneCallback, this, _1, _2),
//      boost::bind(&framefab_gui::PathPlanningState::pathPlanningActiveCallback, this),
//      boost::bind(&framefab_gui::PathPlanningState::pathPlanningFeedbackCallback, this, _1));
//  ROS_INFO_STREAM("Goal sent from path planning state");
  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  f_ac_.waitForServer(); //will wait for infinite time

  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action
  framefab_msgs::FibonacciGoal goal;
  goal.order = 20;
  f_ac_.sendGoal(goal,
                 boost::bind(&framefab_gui::PathPlanningState::fDoneCallback, this, _1, _2),
                 boost::bind(&framefab_gui::PathPlanningState::fActiveCallback, this),
                 boost::bind(&framefab_gui::PathPlanningState::fFeedbackCallback, this, _1));

  //wait for the action to return
  bool finished_before_timeout = f_ac_.waitForResult(ros::Duration(30.0));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = f_ac_.getState();
    ROS_INFO("Action finished: %s", state.toString().c_str());
  }
  else
  {
    ROS_INFO("Action did not finish before the time out.");
  }
}

void framefab_gui::PathPlanningState::setFeedbackText(QString feedback)
{
  gui_ptr_->appendText("\n" + feedback.toStdString());
}

// Action Callbacks
void framefab_gui::PathPlanningState::pathPlanningDoneCallback(
    const actionlib::SimpleClientGoalState& state,
    const framefab_msgs::PathPlanningResultConstPtr& result)
{
  if(result->succeeded)
  {
      ROS_INFO_STREAM("path planning action succeeded");
//    Q_EMIT newStateAvailable(new SelectPlansState());
  }
  else
{
    Q_EMIT newStateAvailable(new SystemInitState());
}
}

void framefab_gui::PathPlanningState::pathPlanningActiveCallback()
{
  ROS_INFO_STREAM("Path Planning Goal is active");
}

void framefab_gui::PathPlanningState::pathPlanningFeedbackCallback(
    const framefab_msgs::PathPlanningFeedbackConstPtr& feedback)
{
  Q_EMIT feedbackReceived(QString::fromStdString((feedback->last_completed).c_str()));
}

// ----------------- Test Fibonacci

void framefab_gui::PathPlanningState::fDoneCallback(
    const actionlib::SimpleClientGoalState& state,
    const framefab_msgs::FibonacciResultConstPtr& result)
{
  ROS_INFO_STREAM("F action done!");
}

void framefab_gui::PathPlanningState::fActiveCallback()
{
  ROS_INFO_STREAM("Path Planning Goal is active");
}

void framefab_gui::PathPlanningState::fFeedbackCallback(const framefab_msgs::FibonacciFeedbackConstPtr& feedback)
{
  Q_EMIT feedbackReceived(QString::number(feedback->sequence.back()));

}
