//
// Created by yijiangh on 11/12/17.
//

#include <framefab_gui/selection/select_for_plan_pop_up_widget.h>
#include <ui_select_for_plan_pop_up.h>

framefab_gui::SelectForPlanPopUpWidget::SelectForPlanPopUpWidget(QWidget* parent) : QWidget(parent)
{
  ui_ = new Ui::SelectForPlanPopUpWidget();
  ui_->setupUi(this);

  connect(ui_->pushbutton_recompute, SIGNAL(clicked()), this, SIGNAL(buttonRecompute()));
  connect(ui_->pushbutton_keep_record, SIGNAL(clicked()), this, SIGNAL(buttonKeepRecord()));

  connect(ui_->pushbutton_recompute, SIGNAL(clicked()), this, SLOT(closeWindow()));
  connect(ui_->pushbutton_keep_record, SIGNAL(clicked()), this, SLOT(closeWindow()));
}

void framefab_gui::SelectForPlanPopUpWidget::setDisplayText(const std::string& text)
{
  ui_->textedit_msg->setPlainText(QString::fromStdString(text));
}

void framefab_gui::SelectForPlanPopUpWidget::enableButtons(bool record_found)
{
  if(record_found)
  {
    ui_->pushbutton_recompute->setEnabled(true);
    ui_->pushbutton_keep_record->setEnabled(true);
  }
  else
  {
    ui_->pushbutton_recompute->setEnabled(true);
    ui_->pushbutton_keep_record->setEnabled(false);
  }
}

void framefab_gui::SelectForPlanPopUpWidget::closeWindow()
{
  this->close();
}

void framefab_gui::SelectForPlanPopUpWidget::closeEvent(QCloseEvent *ev)
{
  Q_EMIT exitSelectForPlanPopUpWidget();
}