#ifndef FRAMEFAB_GUI_FRAMEFAB_WIDGET_H
#define FRAMEFAB_GUI_FRAMEFAB_WIDGET_H

#include <QWidget>

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

#include <framefab_gui/gui_state.h>
#include <framefab_gui/params_submenu.h>
#include <framefab_gui/selection/selection_widget.h>

#include "actionlib/client/simple_action_client.h"

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
 * 1. System Init State
 * 2. Path Planning State           (require computed path info from framefab)
 * 3. Select Path State             (user select path for process planning)
 * 4. Process Planning State        (motion planning and fill in trajectory library)
 * 5. Select Plan State             (User selects plans, simulate and output)
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
  void setParamsButtonEnabled(bool enabled);

  void setLabelText(const std::string& txt);
  void sendGoal(const framefab_msgs::SimulateMotionPlanGoal& goal);
  void sendGoalAndWait(const framefab_msgs::SimulateMotionPlanGoal& goal);

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

  void showOutputPathParams();

 protected:
  // UI
  Ui::FrameFabWidget* ui_;
  ParamsSubmenu* params_;
  SelectionWidget* selection_widget_;

  // ROS specific stuff
  ros::NodeHandle nh_;

  // Current state
  GuiState* active_state_;

  // Params Save
  ros::ServiceClient framefab_parameters_client_;

  actionlib::SimpleActionClient<framefab_msgs::SimulateMotionPlanAction>  simulate_motion_plan_action_client_;
};
}

#endif // FRAMEFAB_GUI_FRAMEFAB_WIDGET_H
