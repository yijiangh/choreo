// QT
#include <QVBoxLayout>
#include <QLabel>
#include <QtGui>
#include <QFuture>
#include <QtConcurrentRun>

// framefab
#include <util/global_functions.h>
#include <framefab_rviz_panel.h>
#include <framefab_render_widget.h>
#include <QtGui/QtGui>

namespace framefab
{

FrameFabRvizPanel::FrameFabRvizPanel( QWidget* parent )
    : rviz::Panel( parent )
{
  ROS_INFO("FrameFabPlanner Rviz Panel started.");

  ptr_ff_render_widget_ = new FrameFabRenderWidget(this);

  createLables();
  createTextEdits();
  createSpinboxes();
  createComboBoxes();
  createPushButtons();
  createPathSlider();
  createGroups();

  QHBoxLayout* topic_layout = new QHBoxLayout;
  topic_layout->addWidget( new QLabel("Input Model File:" ));
  topic_layout->addWidget( pushbutton_readfile_ );

  // Lay out the topic field above the control widget.
  QVBoxLayout* layout = new QVBoxLayout;
  layout->addLayout(topic_layout);
  layout->addWidget(textedit_log_);
  layout->addWidget(groupbox_model_param_);
  layout->addWidget(pushbutton_advancerobot_);
  layout->addWidget(path_slider_);
  setLayout(layout);
}

FrameFabRvizPanel::~FrameFabRvizPanel()
{
  framefab::safeDelete(ptr_ff_render_widget_);
}

void FrameFabRvizPanel::createTextEdits()
{
  textedit_log_ = new QTextEdit(this);
  textedit_log_->setReadOnly(true);
  connect(ptr_ff_render_widget_, SIGNAL(sendLogInfo(QString)), textedit_log_, SLOT(append(QString)));
}

void FrameFabRvizPanel::createLables()
{
  label_scale_    = new QLabel(QString("Model Unit: "), this);
  label_offset_x_ = new QLabel(QString("Offset x: "), this);
  label_offset_y_ = new QLabel(QString("Offset y: "), this);
  label_offset_z_ = new QLabel(QString("Offset z: "), this);
}

void FrameFabRvizPanel::createPushButtons()
{
  pushbutton_readfile_ = new QPushButton("Read File");

  connect(pushbutton_readfile_, SIGNAL(clicked()), this, SLOT(getScaleFactor()));
  connect(this, SIGNAL(sendScaleFactor(QString)), ptr_ff_render_widget_, SLOT(setScaleFactor(QString)));

  connect(pushbutton_readfile_, SIGNAL(clicked()), this, SLOT(getRefPoint()));
  connect(this, SIGNAL(sendRefPoint(double, double, double)),
          ptr_ff_render_widget_, SLOT(setRefPoint(double, double, double)));

  connect(pushbutton_readfile_, SIGNAL(clicked()), ptr_ff_render_widget_, SLOT(readFile()));

  pushbutton_update_unit_scale_ = new QPushButton("Update Model Unit");
  connect(pushbutton_update_unit_scale_, SIGNAL(clicked()), this, SLOT(getUpdatedScaleFactor()));
  connect(this, SIGNAL(updateScaleFactor(QString)),
          ptr_ff_render_widget_, SLOT(setScaleFactor(QString)));

  pushbutton_update_ref_pt_ = new QPushButton("Update Ref Point (Building Ground)");
  connect(pushbutton_update_ref_pt_, SIGNAL(clicked()), this, SLOT(getUpdatedRefPoint()));
  connect(this, SIGNAL(updateRefPoint(double, double, double)),
          ptr_ff_render_widget_, SLOT(setRefPoint(double, double, double)));

  pushbutton_advancerobot_ = new QPushButton("Advance Robot");
  connect(pushbutton_advancerobot_, SIGNAL(clicked()), this, SLOT(advanceRobotButtonHandler()));

  connect(this, SIGNAL(enablePanel(bool)), this, SLOT(enablePanelHandler(bool)));
}

void FrameFabRvizPanel::createPathSlider()
{
  path_slider_ = new QSlider(Qt::Horizontal);
  path_slider_->setTickPosition(QSlider::TicksBothSides);
  path_slider_->setTickInterval(10);
  path_slider_->setSingleStep(1);
  connect( path_slider_, SIGNAL(valueChanged(int)), ptr_ff_render_widget_, SLOT(setValue(int)));
}

void FrameFabRvizPanel::createSpinboxes()
{
  spinbox_offset_x_ = new QDoubleSpinBox(this);
  spinbox_offset_x_->setFixedWidth(140);
  spinbox_offset_x_->setDecimals(1);
  spinbox_offset_x_->setRange(-3000,3000);
  spinbox_offset_x_->setValue(500);
  spinbox_offset_x_->setSingleStep(0.5);

  spinbox_offset_y_ = new QDoubleSpinBox(this);
  spinbox_offset_y_->setFixedWidth(140);
  spinbox_offset_y_->setDecimals(1);
  spinbox_offset_y_->setRange(-3000, 3000);
  spinbox_offset_y_->setValue(0);
  spinbox_offset_y_->setSingleStep(0.5);

  spinbox_offset_z_ = new QDoubleSpinBox(this);
  spinbox_offset_z_->setFixedWidth(140);
  spinbox_offset_z_->setDecimals(1);
  spinbox_offset_z_->setRange(-3000, 3000);
  spinbox_offset_z_->setValue(0);
  spinbox_offset_z_->setSingleStep(0.5);
}

void FrameFabRvizPanel::createComboBoxes()
{
  combobox_scale_ = new QComboBox(this);
  combobox_scale_->addItem("millimeter");
  combobox_scale_->addItem("centimeter");
  combobox_scale_->addItem("inch");
  combobox_scale_->addItem("foot");

}

void FrameFabRvizPanel::createGroups()
{
  groupbox_model_param_ = new QGroupBox(tr("Model Parameters"), this);
  groupbox_model_param_->setFlat(true);
  QVBoxLayout *meshpara_layout = new QVBoxLayout(groupbox_model_param_);
  meshpara_layout->addWidget(label_scale_);
  meshpara_layout->addWidget(combobox_scale_);
  meshpara_layout->addWidget(pushbutton_update_unit_scale_);

  meshpara_layout->addWidget(label_offset_x_);
  meshpara_layout->addWidget(spinbox_offset_x_);
  meshpara_layout->addWidget(label_offset_y_);
  meshpara_layout->addWidget(spinbox_offset_y_);
  meshpara_layout->addWidget(label_offset_z_);
  meshpara_layout->addWidget(spinbox_offset_z_);
  meshpara_layout->addWidget(pushbutton_update_ref_pt_);
}

void FrameFabRvizPanel::getScaleFactor()
{
  Q_EMIT(sendScaleFactor(QString(combobox_scale_->currentText())));
}

void FrameFabRvizPanel::getUpdatedScaleFactor()
{
  Q_EMIT(updateScaleFactor(QString(combobox_scale_->currentText())));
}

void FrameFabRvizPanel::getRefPoint()
{
  Q_EMIT(sendRefPoint(spinbox_offset_x_->value(),
                      spinbox_offset_y_->value(),
                      spinbox_offset_z_->value()));
}

void FrameFabRvizPanel::getUpdatedRefPoint()
{
  Q_EMIT(updateRefPoint(spinbox_offset_x_->value(),
                      spinbox_offset_y_->value(),
                      spinbox_offset_z_->value()));
}

void FrameFabRvizPanel::enablePanelHandler(bool status)
{
  // enable or disable panel
  this->setEnabled(status);
}

void FrameFabRvizPanel::advanceRobotButtonHandler()
{
  // Start advance robot in another thread
  QFuture<void> future = QtConcurrent::run(this, &FrameFabRvizPanel::advanceRobot);
}

void FrameFabRvizPanel::advanceRobot()
{
  // Disable UI
  Q_EMIT(enablePanel(false));

  // let widget call the service
  ptr_ff_render_widget_->advanceRobot();

  // Re-enable UI
  Q_EMIT(enablePanel(true)); // Enable UI
}

void FrameFabRvizPanel::save( rviz::Config config ) const
{
  rviz::Panel::save(config);
}

// Load all configuration data for this panel from the given Config object.
void FrameFabRvizPanel::load( const rviz::Config& config )
{
  rviz::Panel::load(config);
}

} /* namespace */

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(framefab::FrameFabRvizPanel,rviz::Panel)