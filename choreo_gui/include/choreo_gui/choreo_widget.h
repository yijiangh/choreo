#ifndef CHOREO_GUI_CHOREO_WIDGET_H
#define CHOREO_GUI_CHOREO_WIDGET_H

#include <QWidget>

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

#include <choreo_gui/gui_state.h>
#include <choreo_gui/params_submenu.h>
#include <choreo_gui/selection/selection_widget.h>

#include "actionlib/client/simple_action_client.h"

#include "choreo_msgs/SimulateMotionPlanAction.h"
#include "choreo_msgs/SimulateMotionPlanActionGoal.h"

namespace Ui
{
class ChoreoWidget;
}

namespace choreo_gui
{
/**
 * @brief The ChoreoWidget class works in states:
 * 1. System Init State
 * 2. Task Sequence Planning State  (require computed path info from choreo)
 * 2' Task Sequence Processing State
 * 3. Select Path State             (user select path for process planning)
 * 4. Process Planning State        (motion planning and fill in trajectory library)
 * 5. Select Plan State             (User selects plans, simulate and output)
 */
class ChoreoWidget : public QWidget
{
  Q_OBJECT
 public:
  ChoreoWidget(QWidget* parent = 0);

  virtual ~ChoreoWidget();

  // Interface for the states to interact with
  void setText(const std::string& txt);
  void appendText(const std::string& txt);

  void setButtonsEnabled(bool enabled);
  void setParamsButtonEnabled(bool enabled);

  void setLabelText(const std::string& txt);
  void sendGoal(const choreo_msgs::SimulateMotionPlanGoal& goal);
  void sendGoalAndWait(const choreo_msgs::SimulateMotionPlanGoal& goal);

  ros::NodeHandle&  nodeHandle() { return nh_; }
  ParamsSubmenu&    params() { return *params_; }
  SelectionWidget&  selection_widget() { return *selection_widget_; }

 protected:
  void loadParameters();

 protected Q_SLOTS:
  // Button Handlers
  void onNextButton();
  void onBackButton();
  void onResetButton();
  void onParamsButton();

  void onEnableButtons();
  void onDisableButtons();

  void onParamsSave();
  void onParamsAccept();

  // State Change
  void changeState(GuiState* new_state);

  void showOutputSaveDirParams();

  void onUseSavedResult(bool);

  // Pick n' place functionality
  void onPicknPlace();

 protected:
  // UI
  Ui::ChoreoWidget* ui_;
  ParamsSubmenu* params_;
  SelectionWidget* selection_widget_;

  // ROS specific stuff
  ros::NodeHandle nh_;

  // Current state
  GuiState* active_state_;

  // Params Save
  ros::ServiceClient choreo_parameters_client_;

  actionlib::SimpleActionClient<choreo_msgs::SimulateMotionPlanAction>  simulate_motion_plan_action_client_;
};
}

#endif // CHOREO_GUI_CHOREO_WIDGET_H
