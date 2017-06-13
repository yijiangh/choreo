#include <QVBoxLayout>
#include <QLabel>
#include <QMessageBox>
#include <QFuture>
#include <QtConcurrentRun>
#include <QWidget>
#include <QTabWidget>

#include "framefab_rviz_plugin.h"

namespace framefab_rviz_plugin
{
FrameFabRvizPlugin::FrameFabRvizPlugin(QWidget* parent) :
    rviz::Panel(parent)
{
  // Create Tabs
  tab_widget_ = new QTabWidget();

  path_planning_widget_ = new PathPlanningWidget();
  post_processor_widget_ = new PostProcessorWidget();

  tab_widget_->addTab(path_planning_widget_, "Path planning");
  tab_widget_->addTab(post_processor_widget_, "Post processor");
  tab_widget_->setTabEnabled(0, true);
  tab_widget_->setTabEnabled(1, false);
  tab_widget_->setTabEnabled(2, false);
  tab_widget_->setTabEnabled(3, false);
  tab_widget_->setTabEnabled(4, false);

  // Bottom status layout
  QVBoxLayout* status_layout = new QVBoxLayout;
  status_layout->addWidget(new QLabel("Status:"));

  // Global Layout
  QVBoxLayout* global_layout = new QVBoxLayout;
  global_layout->addWidget(tab_widget_);
  global_layout->addLayout(status_layout);
  status_label_ = new QLabel;
  global_layout->addWidget(status_label_);
  setLayout(global_layout);

  // Connect handlers
  // PATH PLANNING
  // Will display a status in general status label ( from path_planning widget )
  connect(path_planning_widget_, SIGNAL(sendStatus(QString)), this, SLOT(displayStatusHandler(QString)));
  connect(path_planning_widget_, SIGNAL(sendMsgBox(QString, QString , QString)), this,
          SLOT(displayMsgBoxHandler(QString, QString, QString)));
  // Call configChanged at each time that path_planning_widget is modified
  connect(path_planning_widget_, SIGNAL(guiChanged()), this, SLOT(triggerSave()));
  // Enable path_planning_widget when comparison_widget_ will send the SIGNAL
  connect(path_planning_widget_, SIGNAL(enablePanelPostProcessor()), this, SLOT(enablePanelPostProcessorHandler()));
  // Enable general panel when path_planning send the SIGNAL
  connect(path_planning_widget_, SIGNAL(enablePanel(bool)), this, SLOT(enablePanelHandler(bool)));
  // Received a signal from comparison widget in order to get CAD and scan params
  connect(path_planning_widget_, SIGNAL(getCADAndScanParams()), this, SLOT(sendCADAndScanDatasSlot()));
  // Send a signal to comparison widget in order to give CAD and scan params
  connect(this, SIGNAL(sendCADAndScanDatas(const QString, const QString)),
  path_planning_widget_, SLOT(setCADAndScanParams(const QString, const QString)));

  //POST_PROCESSOR
  // Will display a status in general status label ( from post_processor widget )
  connect(post_processor_widget_, SIGNAL(sendStatus(QString)), this, SLOT(displayStatusHandler(QString)));
  connect(post_processor_widget_, SIGNAL(sendMsgBox(QString, QString, QString)), this,
          SLOT(displayMsgBoxHandler(QString, QString, QString)));
  // Call configChanged at each time that post_processor_widget is modified
  connect(post_processor_widget_, SIGNAL(guiChanged()), this, SLOT(triggerSave()));
  // Enable general panel when post_processor send the SIGNAL
  connect(post_processor_widget_, SIGNAL(enablePanel(bool)), this, SLOT(enablePanelHandler(bool)));
  // Receive a signal from post processor widget in order to send robot poses data
  connect(post_processor_widget_, SIGNAL(getRobotTrajectoryData()), this, SLOT(setRobotTrajectoryData()));

  connect(this, SIGNAL(displayStatus(const QString)), this, SLOT(displayStatusHandler(const QString)));
}

FrameFabRvizPlugin::~FrameFabRvizPlugin()
{}

void FrameFabRvizPlugin::enablePanelAlignmentHandler()
{
  tab_widget_->setTabEnabled(1, true);
}

void FrameFabRvizPlugin::enablePanelComparisonHandler()
{
  tab_widget_->setTabEnabled(2, true);
}

void FrameFabRvizPlugin::enablePanelPathPlanningHandler()
{
  tab_widget_->setTabEnabled(3, true);
}

void FrameFabRvizPlugin::enablePanelPostProcessorHandler()
{
  tab_widget_->setTabEnabled(4, true);
}

void FrameFabRvizPlugin::enablePanelHandler(const bool status)
{
  setEnabled(status);
}

void FrameFabRvizPlugin::displayStatusHandler(const QString message)
{
  status_label_->setText(message);
}

void FrameFabRvizPlugin::displayMsgBoxHandler(const QString title, const QString msg, const QString info_msg)
{
  enablePanelHandler(false);
  QMessageBox msg_box;
  msg_box.setWindowTitle(title);
  msg_box.setText(msg);
  msg_box.setInformativeText(info_msg);
  msg_box.setIcon(QMessageBox::Critical);
  msg_box.setStandardButtons(QMessageBox::Ok);
  msg_box.exec();
  enablePanelHandler(true);
}

void FrameFabRvizPlugin::triggerSave()
{
  Q_EMIT configChanged();
}

void FrameFabRvizPlugin::setCADDatas(const QString cad_filename)
{
  cad_filename_ = cad_filename;
}

void FrameFabRvizPlugin::setScanDatas(const QString scan_filename)
{
  scan_filename_ = scan_filename;
}

void FrameFabRvizPlugin::setRobotTrajectoryData()
{
  post_processor_widget_->setRobotPoses(path_planning_widget_->getRobotPoses());
  post_processor_widget_->setIsGrindingPose(path_planning_widget_->getIsGrindingPose());
}

void FrameFabRvizPlugin::sendCADAndScanDatasSlot()
{
  Q_EMIT sendCADAndScanDatas(cad_filename_, scan_filename_);
}

// Save all configuration data from this panel to the given Config object
void FrameFabRvizPlugin::save(rviz::Config config) const
{
  rviz::Panel::save(config);

  scanning_widget_->save(config);
  alignment_widget_->save(config);
  comparison_widget_->save(config);
  path_planning_widget_->save(config);
  post_processor_widget_->save(config);
}

// Load all configuration data for this panel from the given Config object.
void FrameFabRvizPlugin::load(const rviz::Config& config)
{
  rviz::Panel::load(config);

  scanning_widget_->load(config);
  alignment_widget_->load(config);
  comparison_widget_->load(config);
  path_planning_widget_->load(config);
  post_processor_widget_->load(config);
}

}  // end namespace

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(framefab_rviz_plugin::FrameFabRvizPlugin, rviz::Panel)