#include <ros/console.h>

#include "choreo_gui/choreo_panel.h"
#include "choreo_gui/choreo_widget.h"

#include <QVBoxLayout>

choreo_gui::ChoreoPanel::ChoreoPanel(QWidget* parent) : rviz::Panel(parent)
{
  QVBoxLayout* layout = new QVBoxLayout(this);
  widget_ = new ChoreoWidget();

  layout->addWidget(widget_);
  setLayout(layout);

  ROS_INFO("[UI] Loaded choreo rviz panel");
}

choreo_gui::ChoreoPanel::~ChoreoPanel() {}

void choreo_gui::ChoreoPanel::onInitialize()
{
  ROS_INFO("[UI] Initializing choreo panel");
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(choreo_gui::ChoreoPanel, rviz::Panel)