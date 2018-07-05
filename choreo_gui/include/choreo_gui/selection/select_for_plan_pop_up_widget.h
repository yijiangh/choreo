//
// Created by yijiangh on 11/12/17.
//

#ifndef CHOREO_GUI_SELECT_FOR_PLAN_POP_UP_WIDGET_H
#define CHOREO_GUI_SELECT_FOR_PLAN_POP_UP_WIDGET_H

#include <QWidget>

namespace Ui
{
class SelectForPlanPopUpWidget;
}

namespace choreo_gui
{

class SelectForPlanPopUpWidget : public QWidget
{
  Q_OBJECT

 public:
  SelectForPlanPopUpWidget(QWidget* parent = 0);

  void setFileName(std::string s) { file_name_ = s; }

  void setDisplayText(const std::string& text);

  void enableButtons(bool record_found, bool enable_recompute = true);

  Q_SIGNALS:
  void buttonRecompute();
  void buttonKeepRecord();
  void exitSelectForPlanPopUpWidget();

 protected Q_SLOTS:
  void closeWindow();

 protected:
  virtual void closeEvent(QCloseEvent *ev);

 private:
  // Display layout
  Ui::SelectForPlanPopUpWidget* ui_;

  std::string file_name_;
};
} // end namespace choreo_gui

#endif //CHOREO_GUI_SELECT_FOR_PLAN_POP_UP_WIDGET_H