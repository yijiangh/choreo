#ifndef PARAMS_SUBMENU_H
#define PARAMS_SUBMENU_H

#include <QWidget>

#include "framefab_gui/options/robot_scan_configuration.h"
#include "framefab_gui/options/surface_detection_configuration.h"
#include "framefab_gui/options/scan_tool_configuration.h"
#include "framefab_gui/options/path_planning_configuration.h"

namespace Ui
{
class ParamsSubmenu;
}

namespace framefab_gui
{

class ParamsSubmenu : public QWidget
{
  Q_OBJECT

 public:
  ParamsSubmenu(QWidget* parent = 0);

  // For every submenu / set of parameters, we have getters/setters
  const godel_msgs::RobotScanParameters& robotScanParams() const;
  void setRobotScanParams(const godel_msgs::RobotScanParameters& params);

  const godel_msgs::SurfaceDetectionParameters& surfaceDetectionParams() const;
  void setSurfaceDetectionParams(const godel_msgs::SurfaceDetectionParameters& params);

  const godel_msgs::PathPlanningParameters& pathPlanningParams() const;
  void setPathPlanningParams(const godel_msgs::PathPlanningParameters& params);

  const godel_msgs::ScanPlanParameters& scanParams() const;
  void setScanParams(const godel_msgs::ScanPlanParameters& params);

  Q_SIGNALS:
  void saveRequested();

 private:
  // Display layout
  Ui::ParamsSubmenu* ui_;
  // Configuration components
  RobotScanConfigWidget* robot_scan_;
  SurfaceDetectionConfigWidget* surface_detection_;
  PathPlanningConfigWidget* path_planning_params_;
  ScanPlanConfigWidget* scan_params_;
};

} // end namespace framefab_gui

#endif // PARAMS_SUBMENU_H
