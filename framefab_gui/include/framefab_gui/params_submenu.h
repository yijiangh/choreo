#ifndef PARAMS_SUBMENU_H
#define PARAMS_SUBMENU_H

#include <QWidget>

#include "framefab_gui/params/model_input_config_widget.h"
#include "framefab_gui/params/path_input_config_widget.h"

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
  const framefab_msgs::ModelInputParameters& modelInputParams() const;
  void  setModelInputParams(const framefab_msgs::ModelInputParameters& params);

  const framefab_msgs::PathInputParameters& pathInputParams() const;
  void setPathInputParams(const framefab_msgs::PathInputParameters& params);

//  const framefab_msgs::PathPlanningParameters& pathPlanningParams() const;
//  void setPathPlanningParams(const framefab_msgs::PathPlanningParameters& params);

  Q_SIGNALS:
  void saveRequested();

 private:
  // Display layout
  Ui::ParamsSubmenu* ui_;

  // Configuration components
  ModelInputConfigWidget*   model_input_widget_;
  PathInputConfigWidget*    path_input_widget_;

//  PathPlanningConfigWidget* path_planning_params_;
//  ScanPlanConfigWidget* scan_params_;
};

} // end namespace framefab_gui

#endif // PARAMS_SUBMENU_H