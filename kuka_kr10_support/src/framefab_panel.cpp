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
#include <std_msgs/Bool.h>

#include <deque>
#include "framefab_panel.h"

// RViz plugin for interacting with framefab files and publishing to kuka_node
namespace framefab 
{
   //set up graph rep
static std::vector<geometry_msgs::Point> nodes;
static std::deque<std::pair<int,int> > edges;
static std::vector<std::pair<int,int> > pillars;
static std::vector<std::pair<int,int> > ceilings;

FramefabPanel::FramefabPanel( QWidget* parent )
  : rviz::Panel( parent )
{
  // Next we lay out the "output topic" text entry field using a
  // QLabel and a QLineEdit in a QHBoxLayout.
  seq_file_editor_ = new QLineEdit;
  browse_button_ = new QPushButton("Browse");
  connect( browse_button_ , SIGNAL( clicked() ), this, SLOT( readFile() ));

  QHBoxLayout* topic_layout = new QHBoxLayout;
  topic_layout->addWidget( new QLabel( "Sequence File:" ));
  topic_layout->addWidget( seq_file_editor_ );
  topic_layout->addWidget( browse_button_ );

  // Lay out the topic field above the control widget.
  file_display_ = new QTextEdit;
  publishlink_button_ = new QPushButton("Publish links");
  connect( publishlink_button_, SIGNAL( clicked() ), this, SLOT( drawLink() ));

  startplan_button_ = new QPushButton("Activate motion planning");
  connect( startplan_button_, SIGNAL( clicked() ), this, SLOT( activateMP() ));

  QVBoxLayout* layout = new QVBoxLayout;
  layout->addLayout( topic_layout );
  layout->addWidget( file_display_ );
  layout->addWidget( publishlink_button_ );
  layout->addWidget( startplan_button_ );
  setLayout( layout );
	
	//todo: the topic queue size (1000, 1, etc) is only for temporal usage, not safe
  pose_publisher_ = nh_.advertise<geometry_msgs::PoseArray>("/framelinks", 1000);
	mplan_publisher_ = nh_advertise<std_msgs::bool>("/activate_mplan", 1);

}

// Read the file name from the dialog and parse results
void FramefabPanel::readFile()
{
      //open file
    QString fileName = QFileDialog::getOpenFileName(this,
         tr("Open File"), "/home/ubuntu/ros_ws/src/kuka_experimental/kuka_kr10_support/data/", tr("pwf Files (*.pwf)"));
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
    QString parse_msg = "Nodes: " + QString::number(nodes.size()) + " Links: " + QString::number( edges.size()) +
		" Pillars: " + QString::number(pillars.size()) + " Ceilings: " + QString::number( ceilings.size());
    file_display_->setText(parse_msg);
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
void FramefabPanel::save( rviz::Config config ) const
{
  rviz::Panel::save( config );
}

// Load all configuration data for this panel from the given Config object.
void FramefabPanel::load( const rviz::Config& config )
{
  rviz::Panel::load( config );
}
 
static geometry_msgs::Point scale(geometry_msgs::Point p, float sf){
  geometry_msgs::Point result;
  result.x = p.x * sf;
  result.y = p.y * sf;
  result.z = p.z * sf;
  return result;
}

//Publishes single link at beginning of edges list and pops off list
void FramefabPanel::drawLink()
{ 
  if (edges.size() == 0 ) {
    ROS_INFO("no links to draw");
    return;
  }
  std::cout << "Publishing Link: " << edges[0].first << " " << edges[0].second << std::endl;
  geometry_msgs::PoseArray msg;
  std::pair<int,int> edge = edges[0];
  geometry_msgs::Pose pose_a;
  geometry_msgs::Pose pose_b;
  pose_a.position = scale(nodes[edge.first], 0.001);
  pose_b.position = scale(nodes[edge.second], 0.001);
  
  msg.poses.push_back( pose_a);
  msg.poses.push_back( pose_b); 
  std::cout << "Publishing points: " << nodes[edge.first] << " " <<  nodes[edge.second] << std::endl;
  edges.pop_front();
  pose_publisher_.publish(msg);
} 

void FramefabPanel::activateMP()
{
	std_msgs::bool mp_msg = true;
	mplan_publisher_.publish(mp_msg);
}

} //namespace
// Tell pluginlib about this class.  Every class which should be
// loadable by pluginlib::ClassLoader must have these two lines
// compiled in its .cpp file, outside of any namespace scope.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(framefab::FramefabPanel,rviz::Panel)
