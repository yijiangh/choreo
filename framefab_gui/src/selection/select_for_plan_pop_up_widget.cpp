//
// Created by yijiangh on 11/12/17.
//

#include <framefab_gui/selection/select_for_plan_pop_up_widget.h>
#include <ui_select_for_plan_pop_up.h>

framefab_gui::SelectForPlanPopUpWidget::SelectForPlanPopUpWidget(QWidget* parent) : QWidget(parent)
{
  ui_ = new Ui::SelectForPlanPopUpWidget();
  ui_->setupUi(this);

//  connect(output_save_dir_input_widget_, SIGNAL(parameters_changed()), this, SIGNAL(acceptRequested()));
}