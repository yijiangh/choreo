/*
 * FrameFabRvizPanel.h
 *
 * Created on:  April 7, 2017
 * Author:      Thomas Cook, Yijiang Huang
 * Institute:   MIT, Digital Structure Group, Building Tech
*/

#ifndef FRAMEFABRVIZPANEL_H
#define FRAMEFABRVIZPANEL_H

// Qt
#include <QObject>
#include <QLineEdit>
#include <QPushButton>
#include <QTextEdit>
#include <QSpinBox>
#include <QString>
#include <QLabel>
#include <QComboBox>
#include <QGroupBox>

// Rviz
#include <rviz/panel.h>

// ROS
#include <ros/ros.h>

// framefab
#include <framefab_render_widget.h>

namespace framefab
{

//! @class FrameFabRvizPanel
/*!
 * @brief framefab UI panel for Rviz
 *
 * This class create UI button, slider and text panel
 * etc. for Framefab control panel on Rviz. The host
 * node for this class is Rviz node. Thus the FrameFab-
 * RvizPanel class is a UI container class connecting
 * Qt object with slot effect functions.
 */
class FrameFabRvizPanel: public rviz::Panel
{
 Q_OBJECT
 public:
  /*!
   * @brief Constructor
   */
  FrameFabRvizPanel( QWidget* parent = 0 );

  /*!
   * @brief Destructor
  */
  ~FrameFabRvizPanel();

  // Now we declare overrides of rviz::Panel functions for saving and
  // loading data from the config file.  Here the data is the
  // topic name.
  virtual void load( const rviz::Config& config );
  virtual void save( rviz::Config config ) const;

 private:
  void createTextEdits();
  void createLables();
  void createPushButtons();
  void createSpinboxes();
  void createComboBoxes();
  void createPathSlider();
  void createGroups();

 Q_SIGNALS:
  void sendScaleFactor(QString);
  void updateScaleFactor(QString);

  void sendRefPoint(double, double, double);
  void updateRefPoint(double, double, double);

  void enablePanel(bool);

 public Q_SLOTS:
  void getScaleFactor();
  void getUpdatedScaleFactor();

  void getRefPoint();
  void getUpdatedRefPoint();

  void advanceRobot();
  void testDescartes();

  protected Q_SLOTS:
  void enablePanelHandler(bool status);
  void advanceRobotButtonHandler();
  void testDescartesButtonHandler();

 protected:
  // TODO: replace raw pointers with shared_ptr

  //! Qt TextEdit - Point display
  QTextEdit* textedit_log_;

  //! Qt Labels
  QLabel* label_scale_;
  QLabel* label_offset_x_;
  QLabel* label_offset_y_;
  QLabel* label_offset_z_;

  //! Qt Pushbuttons
  QPushButton* pushbutton_readfile_;
  QPushButton* pushbutton_update_ref_pt_;
  QPushButton* pushbutton_update_unit_scale_;
  QPushButton* pushbutton_advance_robot_;
  QPushButton* pushbutton_test_descartes_;

  //! Qt Spinbox
  QDoubleSpinBox* spinbox_offset_x_;
  QDoubleSpinBox* spinbox_offset_y_;
  QDoubleSpinBox* spinbox_offset_z_;

  //! Qt ComboBox
  QComboBox* combobox_scale_;

  //! Qt Slider
  QSlider* path_slider_;

  //! Qt Groupboxes
  QGroupBox* groupbox_model_param_;

  //! FrameFab widget - function level
  FrameFabRenderWidget* ptr_ff_render_widget_;
};
}
#endif // FRAMEFABRVIZPANEL_H