// Util
#include "util/GlobalFunctions.h"

// QT
#include <QVBoxLayout>
#include <QLabel>
#include <QtGui>

// framefab
#include "FrameFabRvizPanel.h"

namespace framefab
{

FrameFabRvizPanel::FrameFabRvizPanel( QWidget* parent )
    : rviz::Panel( parent )
{
  ROS_INFO("FrameFab Rviz Panel started.");
  readParameters();

  ptr_ff_render_widget_ = new FrameFabRenderWidget(this);

  createLineEdits();
  createTextEdits();
  createPushButtons();

  QHBoxLayout* topic_layout = new QHBoxLayout;
  topic_layout->addWidget( new QLabel( "Sequence File:" ));
  topic_layout->addWidget( lineEdit_seqFile_ );
  topic_layout->addWidget( pushbutton_readfile_ );

  // Lay out the topic field above the control widget.
  QVBoxLayout* layout = new QVBoxLayout;
  layout->addLayout( topic_layout );
  layout->addWidget( textEdit_ptDisplay_ );
  layout->addWidget( pushbutton_displaypose_ );
  setLayout( layout );
}

FrameFabRvizPanel::~FrameFabRvizPanel()
{
  framefab::SafeDelete(ptr_ff_render_widget_);
}

void FrameFabRvizPanel::createTextEdits()
{
  textEdit_ptDisplay_ = new QTextEdit;
}

void FrameFabRvizPanel::createLineEdits()
{
  lineEdit_seqFile_ = new QLineEdit;
}

void FrameFabRvizPanel::createPushButtons()
{
  pushbutton_displaypose_ = new QPushButton("Display Poses");
  connect( pushbutton_displaypose_, SIGNAL( clicked() ), ptr_ff_render_widget_, SLOT( displayPoses() ));
//
  pushbutton_readfile_ = new QPushButton("Read File");
  connect( pushbutton_readfile_, SIGNAL( clicked() ), ptr_ff_render_widget_, SLOT( readFile() ));
}

// Save all configuration data from this panel to the given
// Config object.  It is important here that you call save()
// on the parent class so the class id and panel name get saved.
void FrameFabRvizPanel::save( rviz::Config config ) const
{
  rviz::Panel::save( config );
}

// Load all configuration data for this panel from the given Config object.
void FrameFabRvizPanel::load( const rviz::Config& config )
{
  rviz::Panel::load( config );
}


} /* namespace */

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(framefab::FrameFabRvizPanel,rviz::Panel)