//
// Created by yijiangh on 6/22/17.
//
#include <QString>
#include <QFileDialog>

#include <framefab_gui/params/task_sequence_input_config_widget.h>
#include <ui_task_sequence_input_config.h>

framefab_gui::TaskSequenceInputConfigWidget::TaskSequenceInputConfigWidget(framefab_msgs::TaskSequenceInputParameters params)
    : params_(params)
{
  ui_ = new Ui::TaskSequenceInputConfigWindow();
  ui_->setupUi(this);

  connect(ui_->pushbutton_accept, SIGNAL(clicked()), this, SLOT(accept_changes_handler()));
  connect(ui_->pushbutton_cancel, SIGNAL(clicked()), this, SLOT(cancel_changes_handler()));
  connect(ui_->pushbutton_save, SIGNAL(clicked()), this, SLOT(save_changes_handler()));

  connect(ui_->pushbutton_readfile, SIGNAL(clicked()), this, SLOT(get_file_path_handler()));

  last_filepath_ = "/home";
}

void framefab_gui::TaskSequenceInputConfigWidget::update_display_fields()
{
  ui_->lineedit_filepath->setText(QString(params_.file_path.c_str()));
}

void framefab_gui::TaskSequenceInputConfigWidget::update_internal_fields()
{
  params_.file_path = ui_->lineedit_filepath->text().toLocal8Bit().constData();
  last_filepath_ = params_.file_path;
}

void framefab_gui::TaskSequenceInputConfigWidget::get_file_path_handler()
{
  QString filename = QFileDialog::getSaveFileName(
      this,
      tr("Open File"),
      QString::fromStdString(last_filepath_));

  params_.file_path = filename.toLocal8Bit().constData();
  last_filepath_ = filename.toLocal8Bit().constData();

  update_display_fields();
}