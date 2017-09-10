//
// Created by yijiangh on 7/10/17.
//

#include <ros/ros.h>
#include <ros/console.h>

#include <framefab_gui/states/select_plan_state.h>
#include <framefab_gui/states/system_init_state.h>
#include <QtConcurrent/QtConcurrentRun>

#include <framefab_msgs/GetAvailableProcessPlans.h>

const static std::string GET_AVAILABLE_PROCESS_PLANS_SERVICE = "get_available_process_plans";

void framefab_gui::SelectPlanState::onStart(FrameFabWidget& gui)
{
  gui.setText("Select Plan State.\n"
                  "Please select the desired plan to be simulated in selection window.\n"
                  "Click <Simulate> to continue. ");

  ptr_gui_ = &gui;

  ros::ServiceClient client = gui.nodeHandle().serviceClient<framefab_msgs::GetAvailableProcessPlans>(
      GET_AVAILABLE_PROCESS_PLANS_SERVICE);

  framefab_msgs::GetAvailableProcessPlans srv;
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

  connect(&gui.selection_widget(), SIGNAL(flushSimulation()), this, SLOT(triggerSimulation()));

  selectPlan(gui);
}

void framefab_gui::SelectPlanState::onExit(FrameFabWidget& gui) {}

void framefab_gui::SelectPlanState::onNext(FrameFabWidget& gui) {}

void framefab_gui::SelectPlanState::onBack(FrameFabWidget& gui)
{
  gui.selection_widget().cleanUpVisual();
  Q_EMIT newStateAvailable(new SystemInitState());
}

void framefab_gui::SelectPlanState::onReset(FrameFabWidget& gui)
{
  gui.selection_widget().cleanUpVisual();
  Q_EMIT newStateAvailable(new SystemInitState());
}

void framefab_gui::SelectPlanState::selectPlan(FrameFabWidget& gui)
{
  gui.selection_widget().setMode(framefab_gui::SelectionWidget::PLAN_SELECTION);
  gui.selection_widget().show();
  gui.selection_widget().loadParameters();
}

void framefab_gui::SelectPlanState::triggerSimulation()
{
  QtConcurrent::run(this, &SelectPlanState::simulateAll);
}

void framefab_gui::SelectPlanState::simulateAll()
{
  selected_plan_ids_.clear();
  selected_plan_ids_ = ptr_gui_->selection_widget().getSelectedIdsForSimulation();

  for (std::size_t i = 0; i < selected_plan_ids_.size(); ++i)
  {
    simulateOne(selected_plan_ids_[i]);
    ROS_INFO_STREAM("[ui] simulate #" << i);
  }

  ptr_gui_->selection_widget().setInputEnabled(true);
}

void framefab_gui::SelectPlanState::simulateOne(const int& plan_id)
{
  framefab_msgs::SimulateMotionPlanGoal goal;
  goal.index = plan_id;
  goal.simulate = false;
  goal.wait_for_execution = false;

  ptr_gui_->sendGoalAndWait(goal);
}

