//
// Created by yijiangh on 9/9/17.
//

#include <QString>
#include <QFileDialog>

#include <framefab_gui/params/output_save_dir_input_config_widget.h>
#include <ui_output_save_dir_input_config.h>

framefab_gui::OutputSaveDirInputConfigWidget::OutputSaveDirInputConfigWidget(framefab_msgs::OutputSaveDirInputParameters params)
    : params_(params)
{
  ui_ = new Ui::OutputSaveDirInputConfigWindow();
  ui_->setupUi(this);

  connect(ui_->pushbutton_accept, SIGNAL(clicked()), this, SLOT(accept_changes_handler()));
  connect(ui_->pushbutton_cancel, SIGNAL(clicked()), this, SLOT(cancel_changes_handler()));
  connect(ui_->pushbutton_save, SIGNAL(clicked()), this, SLOT(save_changes_handler()));

  connect(ui_->pushbutton_output_save_dir, SIGNAL(clicked()), this, SLOT(get_file_path_handler()));

  last_filepath_ = "/home";
}

static void checkFileExtension(std::string& str)
{
  std::string json_suffix = ".json";
  std::size_t found = str.find_last_of(json_suffix);

  while(str.size()-1 == found)
  {
    str = str.substr(0, found - json_suffix.size() + 1);
    found = str.find_last_of(json_suffix);
  }

  str = str + json_suffix;
}

void framefab_gui::OutputSaveDirInputConfigWidget::update_display_fields()
{
  checkFileExtension(params_.file_path);
  ui_->lineedit_filepath->setText(QString(params_.file_path.c_str()));
}

void framefab_gui::OutputSaveDirInputConfigWidget::update_internal_fields()
{
  params_.file_path = ui_->lineedit_filepath->text().toLocal8Bit().constData();
  checkFileExtension(params_.file_path);
}

void framefab_gui::OutputSaveDirInputConfigWidget::get_file_path_handler()
{
  QString filename = QFileDialog::getSaveFileName(
      this,
      tr("Output File"),
      QString::fromStdString(last_filepath_),
      tr("mpp-format json file (*.json)"));

  params_.file_path = filename.toLocal8Bit().constData();
  checkFileExtension(params_.file_path);
  last_filepath_ = params_.file_path;

  update_display_fields();
}
