#include <ros/console.h>

#include "framefab_gui/framefab_panel.h"
#include "framefab_gui/framefab_widget.h"

#include <QVBoxLayout>

framefab_gui::FrameFabPanel::FrameFabPanel(QWidget* parent) : rviz::Panel(parent)
{
  ROS_INFO("Loaded simple blending panel");

  QVBoxLayout* layout = new QVBoxLayout(this);
  widget_ = new FrameFabWidget();
//  mainwindow_ = new MainWindow();
//  mainwindow_->hide();

  layout->addWidget(widget_);
  setLayout(layout);
}

framefab_gui::FrameFabPanel::~FrameFabPanel() {}

void framefab_gui::FrameFabPanel::onInitialize()
{
  ROS_INFO("Initializng framefab panel");
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(framefab_gui::FrameFabPanel, rviz::Panel)