#include <ros/console.h>

#include "framefab_gui/framefab_panel.h"
#include "framefab_gui/framefab_widget.h"

#include <QVBoxLayout>

framefab_gui::FrameFabPanel::FrameFabPanel(QWidget* parent) : rviz::Panel(parent)
{
  QVBoxLayout* layout = new QVBoxLayout(this);
  widget_ = new FrameFabWidget();

  layout->addWidget(widget_);
  setLayout(layout);

  ROS_INFO("[UI] Loaded framefab rviz panel");
}

framefab_gui::FrameFabPanel::~FrameFabPanel() {}

void framefab_gui::FrameFabPanel::onInitialize()
{
  ROS_INFO("[UI] Initializing framefab panel");
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(framefab_gui::FrameFabPanel, rviz::Panel)