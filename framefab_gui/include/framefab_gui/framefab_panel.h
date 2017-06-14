#ifndef FRAMEFAB_PANEL_H
#define FRAMEFAB_PANEL_H

#include <rviz/panel.h>

namespace framefab_gui
{

// Forward declare blend widget
class FrameFabWidget;

class FrameFabPanel : public rviz::Panel
{
  Q_OBJECT
 public:
  FrameFabPanel(QWidget* parent = 0);

  virtual ~FrameFabPanel();

  virtual void onInitialize();

 protected:
  FrameFabWidget* widget_;
};
}

#endif // FRAMEFAB_PANEL_H
