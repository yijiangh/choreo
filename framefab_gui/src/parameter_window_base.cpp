
#include <framefab_gui/parameter_window_base.h>
#include <ros/console.h>

void framefab_gui::ParameterWindowBase::show()
{
  update_display_fields();
  QWidget::show();
}

void framefab_gui::ParameterWindowBase::accept_changes_handler()
{
  this->update_internal_fields();
  Q_EMIT parameters_changed();
  hide();
}

void framefab_gui::ParameterWindowBase::cancel_changes_handler() { hide(); }

void framefab_gui::ParameterWindowBase::save_changes_handler()
{
  accept_changes_handler();
  Q_EMIT parameters_save_requested();
}