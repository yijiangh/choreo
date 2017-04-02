#include <stdio.h>
#include <utility>
#include <QFile>
#include <QPainter>
#include <QTextStream>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>
#include <QFileDialog>
#include <QTextEdit>
#include <geometry_msgs/PoseArray.h>
#include "teleop_panel.h"

namespace framefab 
{

TeleopPanel::TeleopPanel( QWidget* parent )
  : rviz::Panel( parent )
{
  // Next we lay out the "output topic" text entry field using a
  // QLabel and a QLineEdit in a QHBoxLayout.
  QHBoxLayout* topic_layout = new QHBoxLayout;
  topic_layout->addWidget( new QLabel( "Sequence File:" ));
  seq_file_editor_ = new QLineEdit;
  button_ = new QPushButton("Browse");
  topic_layout->addWidget( seq_file_editor_ );
  topic_layout->addWidget( button_ );
  file_display_ = new QTextEdit;
  // Lay out the topic field above the control widget.
  QVBoxLayout* layout = new QVBoxLayout;
  layout->addLayout( topic_layout );
  layout->addWidget( file_display_ );
  setLayout( layout );
  // Create a timer for sending the output.  Motor controllers want to
  // be reassured frequently that they are doing the right thing, so
  // we keep re-sending velocities even when they aren't changing.
  // 
  // Here we take advantage of QObject's memory management behavior:
  // since "this" is passed to the new QTimer as its parent, the
  // QTimer is deleted by the QObject destructor when this TeleopPanel
  // object is destroyed.  Therefore we don't need to keep a pointer
  // to the timer.
  QTimer* output_timer = new QTimer( this );

  // Next we make signal/slot connections.
  connect( button_ , SIGNAL( clicked() ), this, SLOT( readFile() ));
  
  pose_publisher_ = nh_.advertise<geometry_msgs::PoseArray>("/framelinks",1000);
  // Start the timer.
  output_timer->start( 100 );

}



// Read the file name from the dialog and parse results
void TeleopPanel::readFile()
{
    //set up graph rep
    std::vector<geometry_msgs::Point> nodes;
    std::vector<std::pair<int,int> > edges;
    std::vector<std::pair<int,int> > pillars;
    std::vector<std::pair<int,int> > ceilings;
    //open file
    QString fileName = QFileDialog::getOpenFileName(this,
         tr("Open File"), "/home/ubuntu", tr("txt Files (*.pwf)"));
    QFile inputFile(fileName);
    if (inputFile.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        QTextStream in(&inputFile);
	while(!in.atEnd())
	{
	  QString line = in.readLine();
	  QStringList tokens = line.split(" ");
	  if(tokens.at(0) == "v") {
	     geometry_msgs::Point p;
	     p.x = tokens.at(1).toFloat();
   	     p.y = tokens.at(2).toFloat();
	     p.z = tokens.at(3).toFloat();
 	     nodes.push_back(p);
	  } else if (tokens.at(0) == "p") { 
	     pillars.push_back(std::make_pair(tokens.at(1).toInt(), tokens.at(2).toInt()));
	     edges.push_back(std::make_pair(tokens.at(1).toInt(), tokens.at(2).toInt()));
	  } else if (tokens.at(0) == "l") {
             edges.push_back(std::make_pair(tokens.at(1).toInt(), tokens.at(2).toInt()));
	  } else if (tokens.at(0) =="c") {
	     ceilings.push_back(std::make_pair(tokens.at(1).toInt(), tokens.at(2).toInt()));
	     edges.push_back(std::make_pair(tokens.at(1).toInt(), tokens.at(2).toInt()));
	  }
	}
	inputFile.close();
    }
    seq_file_editor_->setText(fileName);
    // rviz::Panel defines the configChanged() signal.  Emitting it
    // tells RViz that something in this panel has changed that will
    // affect a saved config file.  Ultimately this signal can cause
    // QWidget::setWindowModified(true) to be called on the top-level
    // rviz::VisualizationFrame, which causes a little asterisk ("*")
    // to show in the window's title bar indicating unsaved changes.
    Q_EMIT configChanged();
  // Gray out the control widget when the output topic is empty.
}



// Save all configuration data from this panel to the given
// Config object.  It is important here that you call save()
// on the parent class so the class id and panel name get saved.
void TeleopPanel::save( rviz::Config config ) const
{
  rviz::Panel::save( config );
}

// Load all configuration data for this panel from the given Config object.
void TeleopPanel::load( const rviz::Config& config )
{
  rviz::Panel::load( config );
}

} //namespace
// Tell pluginlib about this class.  Every class which should be
// loadable by pluginlib::ClassLoader must have these two lines
// compiled in its .cpp file, outside of any namespace scope.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(framefab::TeleopPanel,rviz::Panel)