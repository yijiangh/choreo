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

// Rviz
#include <rviz/panel.h>

// ROS
#include <ros/ros.h>

// framefab
#include "framefab_render_widget.h"

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


  void console(QString msg) {
    textEdit_ptDisplay_->setText(msg);
  };

 private:
  void createTextEdits();
  void createLineEdits();
  void createPushButtons();
  void createPathSlider();

 protected:
  //! One-line text editor for entering the outgoing ROS topic name.
  QLineEdit* lineEdit_seqFile_;

  //! Point display
  QTextEdit* textEdit_ptDisplay_;

  //! Qt Pushbuttons
  QPushButton* pushbutton_readfile_;
  QPushButton* pushbutton_displaypose_;

  //! Qt Slider
  QSlider* path_slider_;

  //! The ROS node handle.
  ros::NodeHandle nh_;

  //! FrameFab widget - function level
  FrameFabRenderWidget* ptr_ff_render_widget_;


};
}
#endif // FRAMEFABRVIZPANEL_H