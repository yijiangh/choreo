//
// Created by yijiangh on 6/12/17.
//

#ifndef FRAMEFAB_RVIZ_PLUGIN_FRAMEFAB_RVIZ_PLUGIN_H
#define FRAMEFAB_RVIZ_PLUGIN_FRAMEFAB_RVIZ_PLUGIN_H

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <rviz/panel.h>
#include "planning_widget.h"
#include "post_processor_widget.h"
#endif

class QTabWidget;
class QLabel;
class QVBoxLayout;

namespace framefab_rviz_plugin
{
class FrameFabRvizPlugin : public rviz::Panel
{
  Q_OBJECT
 public:
  FrameFabRvizPlugin(QWidget* parent = 0);
  virtual ~FrameFabRvizPlugin();

  Q_SIGNALS:
  void enableWidget(const bool);
  void displayStatus(const QString);
  void sendCADAndScanDatas(const QString, const QString);

 protected Q_SLOTS:
  virtual void triggerSave();
  virtual void load(const rviz::Config& config);
  virtual void save(rviz::Config config) const;

  void displayStatusHandler(const QString message);
  void displayMsgBoxHandler(const QString title, const QString msg, const QString info_msg);

  void enablePanelHandler(const bool);
  void enablePanelAlignmentHandler();
  void enablePanelComparisonHandler();
  void enablePanelPathPlanningHandler();
  void enablePanelPostProcessorHandler();
  void setCADDatas(const QString cad_path);
  void setScanDatas(const QString scan_path);
  void sendCADAndScanDatasSlot();
  void setRobotTrajectoryData();

 protected:
  QString scan_filename_;

  QTabWidget* tab_widget_;
  PlanningWidget*       planning_widget_;
  PostProcessorWidget*  post_processor_widget_;

  QLabel* status_label_;
};

}  // end namespace

#endif //FRAMEFAB_RVIZ_PLUGIN_FRAMEFAB_RVIZ_PLUGIN_H
