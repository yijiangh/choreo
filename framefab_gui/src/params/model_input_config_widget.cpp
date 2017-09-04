//i_path_input_c
// Created by yijiangh on 6/17/17.
//
#include <ros/console.h>
#include <QString>
#include <QFileDialog>

#include <framefab_gui/params/model_input_config_widget.h>
#include <ui_model_input_config.h>

framefab_gui::ModelInputConfigWidget::ModelInputConfigWidget(framefab_msgs::ModelInputParameters params)
    : params_(params)
{
  ui_ = new Ui::ModelInputConfigWindow();
  ui_->setupUi(this);

  connect(ui_->pushbutton_accept, SIGNAL(clicked()), this, SLOT(accept_changes_handler()));
  connect(ui_->pushbutton_cancel, SIGNAL(clicked()), this, SLOT(cancel_changes_handler()));
  connect(ui_->pushbutton_save, SIGNAL(clicked()), this, SLOT(save_changes_handler()));

  connect(ui_->pushbutton_readfile, SIGNAL(clicked()), this, SLOT(get_file_path_handler()));

  ui_->lineedit_ref_pt_x->setValidator(new QDoubleValidator(this));
  ui_->lineedit_ref_pt_y->setValidator(new QDoubleValidator(this));
  ui_->lineedit_ref_pt_z->setValidator(new QDoubleValidator(this));
  ui_->lineedit_element_diameter->setValidator(new QDoubleValidator(this));
  ui_->lineedit_shrink_length->setValidator(new QDoubleValidator(this));

  last_filepath_ = "/home";
}

void framefab_gui::ModelInputConfigWidget::update_display_fields()
{
  ui_->lineedit_filepath->setText(QString(params_.file_name.c_str()));

  ui_->lineedit_ref_pt_x->setText(QString::number(params_.ref_pt_x));
  ui_->lineedit_ref_pt_y->setText(QString::number(params_.ref_pt_y));
  ui_->lineedit_ref_pt_z->setText(QString::number(params_.ref_pt_z));

  ui_->lineedit_element_diameter->setText(QString::number(params_.element_diameter));
  ui_->lineedit_shrink_length->setText(QString::number(params_.shrink_length));

  ui_->combobox_unit->setCurrentIndex(params_.unit_type);
}

void framefab_gui::ModelInputConfigWidget::update_internal_fields()
{
  params_.file_name = ui_->lineedit_filepath->text().toLocal8Bit().constData();

  params_.ref_pt_x = ui_->lineedit_ref_pt_x->text().toDouble();
  params_.ref_pt_y = ui_->lineedit_ref_pt_y->text().toDouble();
  params_.ref_pt_z = ui_->lineedit_ref_pt_z->text().toDouble();

  params_.element_diameter = ui_->lineedit_element_diameter->text().toDouble();
  params_.shrink_length = ui_->lineedit_shrink_length->text().toDouble();

  params_.unit_type = ui_->combobox_unit->currentIndex();
}

int framefab_gui::ModelInputConfigWidget::get_unit_combobox_value()
{
  switch (params_.unit_type)
  {
    case framefab_msgs::ModelInputParameters::MILLIMETER:
      return 0;
    case framefab_msgs::ModelInputParameters::CENTIMETER:
      return 1;
    case framefab_msgs::ModelInputParameters::INCH:
      return 2;
    case framefab_msgs::ModelInputParameters::FOOT:
      return 3;
    case framefab_msgs::ModelInputParameters::METER:
      return 4;
    default:
      return -1;
  }
}

void framefab_gui::ModelInputConfigWidget::get_file_path_handler()
{
  QString filename = QFileDialog::getOpenFileName(
      this,
      tr("Open File"),
      QString::fromStdString(last_filepath_),
      tr("pwf Files (*.pwf)"));

  params_.file_name = filename.toLocal8Bit().constData();
  last_filepath_ = filename.toLocal8Bit().constData();

  update_display_fields();
}