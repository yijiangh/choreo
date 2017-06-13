#ifndef FRAMEFAB_GUI_STATE_H
#define FRAMEFAB_GUI_STATE_H

#include <QObject>

namespace framefab_gui
{

// Forward declare Main Widget
class FrameFabWidget;

class GuiState : public QObject
{
  Q_OBJECT
 public:
  virtual ~GuiState() {}

  // Entry and exit classes
  virtual void onStart(FrameFabWidget& gui) = 0;
  virtual void onExit(FrameFabWidget& gui) = 0;

  // Handlers for the fixed buttons
  virtual void onNext(FrameFabWidget& gui) = 0;
  virtual void onBack(FrameFabWidget& gui) = 0;
  virtual void onReset(FrameFabWidget& gui) = 0;

  Q_SIGNALS:
  void newStateAvailable(GuiState*);
};
}

#endif
