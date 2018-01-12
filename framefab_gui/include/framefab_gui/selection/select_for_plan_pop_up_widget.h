//
// Created by yijiangh on 11/12/17.
//

#ifndef FRAMEFAB_GUI_SELECT_FOR_PLAN_POP_UP_WIDGET_H
#define FRAMEFAB_GUI_SELECT_FOR_PLAN_POP_UP_WIDGET_H

#include <QWidget>

namespace Ui
{
class SelectForPlanPopUpWidget;
}

namespace framefab_gui
{

class SelectForPlanPopUpWidget : public QWidget
{
  Q_OBJECT

 public:
  SelectForPlanPopUpWidget(QWidget* parent = 0);

  void setFileName(std::string s) { file_name_ = s; }

  void setDisplayText(const std::string& text);

  void enableButtons(bool record_found);

  Q_SIGNALS:
  void buttonRecompute();
  void buttonKeepRecord();

 protected Q_SLOTS:
  void closeWindow();

 private:
  // Display layout
  Ui::SelectForPlanPopUpWidget* ui_;

  std::string file_name_;
};
} // end namespace framefab_gui

#endif //FRAMEFAB_GUI_SELECT_FOR_PLAN_POP_UP_WIDGET_H