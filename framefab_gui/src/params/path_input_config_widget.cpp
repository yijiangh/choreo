//
// Created by yijiangh on 6/22/17.
//
#include <QString>
#include <QFileDialog>

#include <framefab_gui/params/path_input_config_widget.h>
#include <ui_path_input_config.h>

framefab_gui::PathInputConfigWidget::PathInputConfigWidget(framefab_msgs::PathInputParameters params)
    : params_(params)
{
  ui_ = new Ui::PathInputConfigWindow();
  ui_->setupUi(this);

  connect(ui_->pushbutton_accept, SIGNAL(clicked()), this, SLOT(accept_changes_handler()));
  connect(ui_->pushbutton_cancel, SIGNAL(clicked()), this, SLOT(cancel_changes_handler()));
  connect(ui_->pushbutton_save, SIGNAL(clicked()), this, SLOT(save_changes_handler()));

  connect(ui_->pushbutton_readfile, SIGNAL(clicked()), this, SLOT(get_file_path_handler()));
}

void framefab_gui::PathInputConfigWidget::update_display_fields()
{
  ui_->lineedit_filepath->setText(QString(params_.file_path.c_str()));
}

void framefab_gui::PathInputConfigWidget::update_internal_fields()
{
  params_.file_path = ui_->lineedit_filepath->text().toLocal8Bit().constData();
}

void framefab_gui::PathInputConfigWidget::get_file_path_handler()
{
  QString filename = QFileDialog::getOpenFileName(
      this,
      tr("Open File"),
      "$HOME/Documents",
      tr("pwf Files (*.pwf)"));

  params_.file_path = filename.toLocal8Bit().constData();
  update_display_fields();
}