#ifndef CHOREO_GUI_STATE_H
#define CHOREO_GUI_STATE_H

#include <QObject>

namespace choreo_gui
{

// Forward declare Main Widget
class ChoreoWidget;

class GuiState : public QObject
{
  Q_OBJECT
 public:
  virtual ~GuiState() {}

  // Entry and exit classes
  virtual void onStart(ChoreoWidget& gui) = 0;
  virtual void onExit(ChoreoWidget& gui) = 0;

  // Handlers for the fixed buttons
  virtual void onNext(ChoreoWidget& gui) = 0;
  virtual void onBack(ChoreoWidget& gui) = 0;
  virtual void onReset(ChoreoWidget& gui) = 0;

  Q_SIGNALS:
  void newStateAvailable(GuiState*);
};
}

#endif
