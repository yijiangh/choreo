#include <utility>

// QT
#include <QFile>
#include <QPainter>
#include <QTextStream>
#include <QVBoxLayout>
#include <QLabel>
#include <QTimer>
#include <QFileDialog>
#include <QTextEdit>
#include <QtGui>

// ROS msgs
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Bool.h>

#include <deque>
#include <QtCore/QtCore>
#include <QtCore/QFile>

// framefab
#include "FrameFabRvizPanel.h"

namespace framefab
{

    FrameFabRvizPanel::FrameFabRvizPanel( QWidget* parent )
            : rviz::Panel( parent )
    {
        ROS_INFO("FrameFab Rviz Panel started.");
        readParameters();

        createLineEdits();
        createTextEdits();
        createPushButtons();

        QHBoxLayout* topic_layout = new QHBoxLayout;
        topic_layout->addWidget( new QLabel( "Sequence File:" ));
        topic_layout->addWidget( lineEdit_seqFile_ );
        topic_layout->addWidget( pushButton_browse_ );

        // Lay out the topic field above the control widget.
        QVBoxLayout* layout = new QVBoxLayout;
        layout->addLayout( topic_layout );
        layout->addWidget( textEdit_ptDisplay_ );
        layout->addWidget( pushButton_publishLink_ );
        layout->addWidget( pushButton_startPlan_ );
        setLayout( layout );

        if(!display_pose_topic_.empty())
        {
            display_pose_publisher_ = node_handle_.advertise<std_msgs::Bool>(display_pose_topic_, 1);
        }

        if(!read_file_topic_.empty())
        {
            read_file_publisher_    = node_handle_.advertise<std_msgs::Bool>(read_file_topic_, 1);
        }
    }

    bool FrameFabRvizPanel::readParameters()
    {
        //FrameFab Parameters
        node_handle_.param("display_pose_topic", display_pose_topic_, std::string("/framelinks"));
        node_handle_.param("read_file_topic", read_file_topic_, std::string("/readfile"));

        return true;
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
        pushButton_publishLink_ = new QPushButton("Publish links");
        connect( pushButton_publishLink_, SIGNAL( clicked() ), this, SLOT( drawLink() ));

        pushButton_startPlan_ = new QPushButton("Activate motion planning");
        connect( pushButton_startPlan_, SIGNAL( clicked() ), this, SLOT( activateMP() ));
    }

    void FrameFabRvizPanel::readFile()
    {
        QString filename = QFileDialog::getOpenfileName(
                this,
                tr("Open File"),
                "/home/ubuntu/ros_ws/src/kuka_experimental/kuka_kr10_support/data/",
                tr("pwf Files (*.pwf)"));

        if(filename.isEmpty())
        {
            ROS_ERROR("Read Model Failed!");
            return;
        }

        // compatible with paths in Chinese
        QTextCodec *code = QTextCodec::codecForName("gd18030");
        QTextCodec::setCodecForLocale(code);
        QByteArray byfilename = filename.toLocal8Bit();

        QFile inputFile(filename);
        if (inputFile.open(QIODevice::ReadOnly | QIODevice::Text)) {
            QTextStream in(&inputFile);
            while (!in.atEnd()) {
                QString line = in.readLine();
                QStringList tokens = line.split(" ");
                if (tokens.at(0) == "v") {
                    geometry_msgs::Point p;
                    p.x = tokens.at(1).toFloat();
                    p.y = tokens.at(2).toFloat();
                    p.z = tokens.at(3).toFloat();
                    nodes_.push_back(p);
                } else if (tokens.at(0) == "p") {
                    pillars_.push_back(std::make_pair(tokens.at(1).toInt(), tokens.at(2).toInt()));
                    edges_.push_back(std::make_pair(tokens.at(1).toInt(), tokens.at(2).toInt()));
                } else if (tokens.at(0) == "l") {
                    edges_.push_back(std::make_pair(tokens.at(1).toInt(), tokens.at(2).toInt()));
                } else if (tokens.at(0) == "c") {
                    ceilings_.push_back(std::make_pair(tokens.at(1).toInt(), tokens.at(2).toInt()));
                    edges_.push_back(std::make_pair(tokens.at(1).toInt(), tokens.at(2).toInt()));
                }
            }
            inputFile.close();
        }

        // emit file info
        lineEdit_seqFile_->setText(filename);
        QString parse_msg = "Nodes: "       + QString::number(nodes_.size())    + "\n"
                            + " Links: "    + QString::number(edges_.size())    + "\n"
                            + " Pillars: "  + QString::number(pillars_.size())  + "\n"
                            + " Ceilings: " + QString::number(ceilings_.size());
        textEdit_ptDisplay_->setText(parse_msg);

        // rviz::Panel defines the configChanged() signal.  Emitting it
        // tells RViz that something in this panel has changed that will
        // affect a saved config file.  Ultimately this signal can cause
        // QWidget::setWindowModified(true) to be called on the top-level
        // rviz::VisualizationFrame, which causes a little asterisk ("*")
        // to show in the window's title bar indicating unsaved changes.
        Q_EMIT configChanged();
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

    geometry_msgs::Point FrameFabRvizPanel::scale(geometry_msgs::Point p, float sf)
    {
        geometry_msgs::Point result;
        result.x = p.x * sf;
        result.y = p.y * sf;
        result.z = p.z * sf;
        return result;
    }


//Publishes single link at beginning of edges_ list and pops off list
    void FrameFabRvizPanel::drawLink()
    {
        if (edges_.size() == 0 ) {
            ROS_INFO("no links to draw");
            return;
        }
        std::cout << "Publishing Link: " << edges_[0].first << " " << edges_[0].second << std::endl;
        geometry_msgs::PoseArray msg;
        std::pair<int,int> edge = edges_[0];
        geometry_msgs::Pose pose_a;
        geometry_msgs::Pose pose_b;
        pose_a.position = scale(nodes_[edge.first], 0.001);
        pose_b.position = scale(nodes_[edge.second], 0.001);

        msg.poses.push_back( pose_a);
        msg.poses.push_back( pose_b);
        std::cout << "Publishing points: " << nodes_[edge.first] << " " <<  nodes_[edge.second] << std::endl;
        edges_.pop_front();

        display_pose_publisher_.publish(msg);
        ROS_DEBUG("MSG: link pose visualize has been published");
    }

} //namespace

// Tell pluginlib about this class.  Every class which should be
// loadable by pluginlib::ClassLoader must have these two lines
// compiled in its .cpp file, outside of any namespace scope.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(framefab::FrameFabRvizPanel,rviz::Panel)
