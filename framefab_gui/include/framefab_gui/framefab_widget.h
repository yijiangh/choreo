#ifndef FRAMEFAB_GUI_FRAMEFAB_WIDGET_H
#define FRAMEFAB_GUI_FRAMEFAB_WIDGET_H

#include <QWidget>

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

#include <framefab_gui/gui_state.h>
#include "framefab_gui/params_submenu.h"

#include "framefab_msgs/SimulateMotionPlanAction.h"
#include "framefab_msgs/SimulateMotionPlanActionGoal.h"

namespace Ui
{
class FrameFabWidget;
}

namespace framefab_gui
{
/**
 * @brief The FrameFabWidget class works in states:
 * 1. Model Input State
 * 2. Planning State
 * 3. (Motion Compensator State) (TODO)
 * 4. PostProcessing State
 */
class FrameFabWidget : public QWidget
{
  Q_OBJECT
 public:
  FrameFabWidget(QWidget* parent = 0);

  virtual ~FrameFabWidget();

  // Interface for the states to interact with
  void setText(const std::string& txt);
  void appendText(const std::string& txt);
  void setButtonsEnabled(bool enabled);
  void showStatusWindow();
  void setLabelText(const std::string& txt);
  void sendGoal(const framefab_msgs::SimulateMotionPlanActionGoal& goal);
  void sendGoalAndWait(const framefab_msgs::SimulateMotionPlanActionGoal& goal);

  ros::NodeHandle& nodeHandle() { return nh_; }

 protected:
  void loadParameters();

 protected Q_SLOTS:
  // Button Handlers
  void onNextButton();
  void onBackButton();
  void onResetButton();
  void onParamsButton();

  void onParamsSave();

  // State Change
  void changeState(GuiState* new_state);

 protected:
  // UI
  Ui::FrameFabWidget* ui_;
  ParamsSubmenu* params_;

  // ROS specific stuff
  ros::NodeHandle nh_;

  // Current state
  GuiState* active_state_;

  // Params Save
  ros::ServiceClient surface_framefab_parameters_client_;
  actionlib::SimpleActionClient<framefab_msgs::SimulateMotionPlanAction> simulate_motion_plan_action_client_;
};
}

#endif // FRAMEFAB_GUI_FRAMEFAB_WIDGET_H