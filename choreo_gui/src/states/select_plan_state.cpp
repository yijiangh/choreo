//
// Created by yijiangh on 7/10/17.
//

#include <ros/ros.h>
#include <ros/console.h>

#include <choreo_gui/states/select_plan_state.h>
#include <choreo_gui/states/system_init_state.h>
#include <QtConcurrent/QtConcurrentRun>

//
#include <choreo_msgs/GetAvailableProcessPlans.h>
#include <choreo_msgs/OutputProcessPlans.h>

const static std::string GET_AVAILABLE_PROCESS_PLANS_SERVICE = "get_available_process_plans";

void choreo_gui::SelectPlanState::onStart(ChoreoWidget& gui)
{
  gui.setText("Select Plan State.\n"
                  "Please select the desired plan to be simulated in selection window.\n"
                  "Click <Simulate> to continue. ");

  ptr_gui_ = &gui;

  ros::ServiceClient client = gui.nodeHandle().serviceClient<choreo_msgs::GetAvailableProcessPlans>(
      GET_AVAILABLE_PROCESS_PLANS_SERVICE);

  choreo_msgs::GetAvailableProcessPlans srv;
  std::vector<std::string> plan_names;

  // fetch all computed plans from core at start
  if (client.call(srv))
  {
    plan_names = srv.response.names;
  }
  else
  {
    ROS_ERROR_STREAM("[Select Plan State] Could not fetch plan names");
  }

  if (plan_names.empty())
  {
    gui.appendText("failed to fetch any computed plan.\n"
                       "Return to Init State");
    Q_EMIT newStateAvailable(new SystemInitState());
    return;
  }
  else
  {
    gui.selection_widget().addFetchedPlans(plan_names);
  }

  gui.setButtonsEnabled(false);
  selected_plan_ids_.clear();
  sim_speed_ = 1.0;

  connect(&gui.selection_widget(), SIGNAL(flushSimulation()), this, SLOT(triggerSimulation()));
  connect(&gui.selection_widget(), SIGNAL(flushOutputProcess()), this, SLOT(triggerOutputProcess()));

  selectPlan(gui);
}

void choreo_gui::SelectPlanState::onExit(ChoreoWidget& gui) {}

void choreo_gui::SelectPlanState::onNext(ChoreoWidget& gui) {}

void choreo_gui::SelectPlanState::onBack(ChoreoWidget& gui)
{
  gui.selection_widget().cleanUpVisual();
  Q_EMIT newStateAvailable(new SystemInitState());
}

void choreo_gui::SelectPlanState::onReset(ChoreoWidget& gui)
{
  gui.selection_widget().cleanUpVisual();
  Q_EMIT newStateAvailable(new SystemInitState());
}

void choreo_gui::SelectPlanState::selectPlan(ChoreoWidget& gui)
{
  gui.selection_widget().setMode(choreo_gui::SelectionWidget::PLAN_SELECTION);
  gui.selection_widget().show();
  gui.selection_widget().loadParameters();
}

void choreo_gui::SelectPlanState::triggerSimulation()
{
  QtConcurrent::run(this, &SelectPlanState::simulateAll);
}

void choreo_gui::SelectPlanState::triggerOutputProcess()
{
  QtConcurrent::run(this, &SelectPlanState::makeOutputProcessRequest);
}

void choreo_gui::SelectPlanState::simulateAll()
{
  selected_plan_ids_.clear();
  selected_plan_ids_ = ptr_gui_->selection_widget().getSelectedIdsForSimulation();
  sim_speed_ = ptr_gui_->selection_widget().getSimSpeed();

  for (std::size_t i = 0; i < selected_plan_ids_.size(); ++i)
  {
    simulateOne(selected_plan_ids_[i]);
    ROS_INFO_STREAM("[ui] simulate #" << selected_plan_ids_[i]);
  }

  ptr_gui_->selection_widget().setInputEnabled(true);
}

void choreo_gui::SelectPlanState::simulateOne(const int& plan_id)
{
  choreo_msgs::SimulateMotionPlanGoal goal;
  goal.action = choreo_msgs::SimulateMotionPlanGoal::SINGLE_PATH_RUN;
  goal.index = plan_id;
  goal.simulate = true;
  goal.wait_for_execution = true;
  goal.sim_speed = sim_speed_;

  ptr_gui_->sendGoalAndWait(goal);
}

void choreo_gui::SelectPlanState::makeOutputProcessRequest()
{
  choreo_msgs::OutputProcessPlans srv;

  std::vector<int> int_ids = ptr_gui_->selection_widget().getChosenIds();
  for(auto id : int_ids)
  {
    srv.request.names.push_back(std::to_string(id));
  }

  ros::ServiceClient output_process_client =
      ptr_gui_->nodeHandle().serviceClient<choreo_msgs::OutputProcessPlans>(
          "output_process_plans");

  ptr_gui_->setButtonsEnabled(false);
  ptr_gui_->selection_widget().setInputEnabled(false);

  output_process_client.waitForExistence();

  if (output_process_client.call(srv))
  {
    ptr_gui_->selection_widget().setStatusBar("output plans done.", true);
  }
  else
  {
    ptr_gui_->selection_widget().setStatusBar("output plans failed!", false);
    ROS_ERROR_STREAM("unable to output process");
  }

  ptr_gui_->setButtonsEnabled(true);
  ptr_gui_->selection_widget().setInputEnabled(true);
}
