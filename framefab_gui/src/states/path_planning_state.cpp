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

// Class level constants
const static std::string PATH_PLANNING_SERVICE = "surface_detection";

framefab_gui::PathPlanningState::PathPlanningState()
{
  ROS_INFO_STREAM("PathPlanningState Init.");
}

framefab_gui::PathPlanningState::~PathPlanningState()
{
}

void framefab_gui::PathPlanningState::onStart(FrameFabWidget& gui)
{
  gui.setText("PathPlanning State.\n Please input data in parameter widget.\n Click 'Next' to continue after finished.");
//  gui.setButtonsEnabled(false);

  path_client_ =
      gui.nodeHandle().serviceClient<framefab_msgs::PathPlanning>(PATH_PLANNING_SERVICE);

  QtConcurrent::run(this, &PathPlanningState::makeRequest, gui.params().modelInputParams(),
                    gui.params().pathInputParams(), gui);
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
    framefab_msgs::PathInputParameters path_params,
    FrameFabWidget& gui)
{
  framefab_msgs::PathPlanning srv;
  srv.request.action = srv.request.FIND_ONLY;
  srv.request.use_default_parameters = false;
  srv.request.model_params = model_params;
  srv.request.path_params  = path_params;

  if (!path_client_.call(srv))
  {
    ROS_WARN_STREAM("Unable to call path planning service");
    Q_EMIT newStateAvailable(new SystemInitState());
  }
  else
  {
    if(srv.response.model_found && srv.response.paths_found)
    {
        gui.setText("model & path found successfully.");
//      Q_EMIT newStateAvailable(new SurfaceSelectState());
    }
    else
    {
      gui.setText("Model or Paths not found!");
      Q_EMIT newStateAvailable(new SystemInitState());
    }
  }
}
