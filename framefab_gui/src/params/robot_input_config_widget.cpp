//
// Created by yijiangh on 9/4/17.
//

#include <QString>
#include <QFileDialog>

#include <framefab_gui/params/robot_input_config_widget.h>
#include <ui_robot_input_config.h>

framefab_gui::RobotInputConfigWidget::RobotInputConfigWidget(framefab_msgs::RobotInputParameters params)
    : params_(params)
{
  ui_ = new Ui::RobotInputConfigWindow();
  ui_->setupUi(this);

  connect(ui_->pushbutton_accept, SIGNAL(clicked()), this, SLOT(accept_changes_handler()));
  connect(ui_->pushbutton_cancel, SIGNAL(clicked()), this, SLOT(cancel_changes_handler()));
  connect(ui_->pushbutton_save, SIGNAL(clicked()), this, SLOT(save_changes_handler()));

  ui_->lineedit_a1->setValidator(new QDoubleValidator(this));
  ui_->lineedit_a2->setValidator(new QDoubleValidator(this));
  ui_->lineedit_a3->setValidator(new QDoubleValidator(this));
  ui_->lineedit_a4->setValidator(new QDoubleValidator(this));
  ui_->lineedit_a5->setValidator(new QDoubleValidator(this));
  ui_->lineedit_a6->setValidator(new QDoubleValidator(this));

  // set default initial pose value
  params_.init_pose = std::vector<double>(6);
  params_.init_pose[0] = 0.08;
  params_.init_pose[1] = -1.57;
  params_.init_pose[2] = 1.74;
  params_.init_pose[3] = 0.08;
  params_.init_pose[4] = 0.17;
  params_.init_pose[5] = -0.08;
}

void framefab_gui::RobotInputConfigWidget::update_display_fields()
{
  ui_->lineedit_a1->setText(QString::number(params_.init_pose[0]));
  ui_->lineedit_a2->setText(QString::number(params_.init_pose[1]));
  ui_->lineedit_a3->setText(QString::number(params_.init_pose[2]));
  ui_->lineedit_a4->setText(QString::number(params_.init_pose[3]));
  ui_->lineedit_a5->setText(QString::number(params_.init_pose[4]));
  ui_->lineedit_a6->setText(QString::number(params_.init_pose[5]));
}

void framefab_gui::RobotInputConfigWidget::update_internal_fields()
{
  params_.init_pose[0] = ui_->lineedit_a1->text().toDouble();
  params_.init_pose[1] = ui_->lineedit_a2->text().toDouble();
  params_.init_pose[2] = ui_->lineedit_a3->text().toDouble();
  params_.init_pose[3] = ui_->lineedit_a4->text().toDouble();
  params_.init_pose[4] = ui_->lineedit_a5->text().toDouble();
  params_.init_pose[5] = ui_->lineedit_a6->text().toDouble();
}