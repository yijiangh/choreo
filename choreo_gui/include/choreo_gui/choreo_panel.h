#ifndef CHOREO_PANEL_H
#define CHOREO_PANEL_H

#include <rviz/panel.h>

namespace choreo_gui
{

class ChoreoWidget;

class ChoreoPanel : public rviz::Panel
{
  Q_OBJECT
 public:
  ChoreoPanel(QWidget* parent = 0);

  virtual ~ChoreoPanel();

  virtual void onInitialize();

 protected:
  ChoreoWidget* widget_;
};
}

#endif // CHOREO_PANEL_H
