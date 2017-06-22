#ifndef MODEL_INPUT_STATE_H
#define MODEL_INPUT_STATE_H

#include <ros/ros.h>
#include <framefab_gui/gui_state.h>

#include <framefab_msgs/ModelInputParameters.h>
#include <framefab_msgs/PathInputParameters.h>

namespace framefab_gui
{

/**
 * launch model input params widget
 * Model Input, computed results input (parse into framefab_msgs::PathCandidate)
 */
class PathPlanningState : public GuiState
{
  Q_OBJECT
 public:
  PathPlanningState();
  ~PathPlanningState();

 public:
  // Entry and exit classes
  virtual void onStart(FrameFabWidget& gui);
  virtual void onExit(FrameFabWidget& gui);

  // Handlers for the fixed buttons
  virtual void onNext(FrameFabWidget& gui);
  virtual void onBack(FrameFabWidget& gui);
  virtual void onReset(FrameFabWidget& gui);

 private:
  void makeRequest(framefab_msgs::ModelInputParameters model_params,
                   framefab_msgs::PathInputParameters path_params);

 private:
  ros::ServiceClient path_client_;
};
}

#endif