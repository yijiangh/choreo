//
// Created by yijiangh on 6/27/17.
//

#include <ui_select_path_widget.h>
#include <framefab_gui/selection/select_path_widget.h>

// service
#include <framefab_msgs/ElementNumberRequest.h>
#include <framefab_msgs/VisualizeSelectedPath.h>

const static std::string ELEMENT_NUMBER_REQUEST_SERVICE = "element_member_request";
const static std::string VISUALIZE_SELECTED_PATH = "visualize_select_path";

framefab_gui::SelectPathWidget::SelectPathWidget(QWidget* parent) : QWidget(parent)
{
  // UI setup
  ui_ = new Ui::SelectPathWidgetWindow;
  ui_->setupUi(this);

  this->setWindowFlags(Qt::WindowStaysOnTopHint);

  // Wire in buttons
  connect(ui_->pushbutton_select_backward, SIGNAL(clicked()), this, SLOT(buttonBackwardUpdateOrderValue()));
  connect(ui_->pushbutton_select_forward, SIGNAL(clicked()), this, SLOT(buttonForwardUpdateOrderValue()));
  connect(ui_->pushbutton_accept, SIGNAL(clicked()), this, SIGNAL(acceptSelection()));

  // Wire in slider
  connect(ui_->slider_select_number, SIGNAL(valueChanged(int)), this, SLOT(sliderUpdateOrderValue(int)));

  // Wire in lineedit
  connect(ui_->lineedit_select_number, SIGNAL(textChanged(QString)), this, SLOT(lineeditUpdateOrderValue(QString)));

  // Start Service Client
  visualize_client_ =
      nh_.serviceClient<framefab_msgs::VisualizeSelectedPath>(VISUALIZE_SELECTED_PATH);
}

void framefab_gui::SelectPathWidget::loadParameters()
{
  framefab_msgs::ElementNumberRequest srv;
  ros::ServiceClient element_number_param_client =
      nh_.serviceClient<framefab_msgs::ElementNumberRequest>(ELEMENT_NUMBER_REQUEST_SERVICE);

  setInputEnabled(false);

  element_number_param_client.waitForExistence();

  if (element_number_param_client.call(srv))
  {
    this->setMaxValue(srv.response.element_number);

    ROS_INFO_STREAM("select path panel fetch model info successfully.");
  }
  else
  {
    ROS_ERROR_STREAM("Unable to fetch model's element number!");
  }

  setInputEnabled(true);
}

void framefab_gui::SelectPathWidget::setMaxValue(int m)
{
  max_value_ = m;

  ui_->slider_select_number->setMaximum(max_value_);
  ui_->lineedit_select_number->setValidator(new QIntValidator(0, max_value_, this));
  ui_->lineedit_max->setText(QString::number(max_value_));
}

void framefab_gui::SelectPathWidget::orderValueChanged()
{
  ui_->slider_select_number->setValue(print_order_);
  ui_->lineedit_select_number->setText(QString::number(print_order_));

  // call visualization srv
  framefab_msgs::VisualizeSelectedPath srv;
  srv.request.index = print_order_;

  setInputEnabled(false);

  visualize_client_.waitForExistence();
  if (!visualize_client_.call(srv))
  {
    ROS_ERROR_STREAM("Unable to visualize selected path!!");
  }

  setInputEnabled(true);
}

void framefab_gui::SelectPathWidget::setInputEnabled(bool enabled)
{
  ui_->pushbutton_select_backward->setEnabled(enabled);
  ui_->pushbutton_select_forward->setEnabled(enabled);
  ui_->pushbutton_accept->setEnabled(enabled);

  ui_->slider_select_number->setEnabled(enabled);
  ui_->lineedit_select_number->setEnabled(enabled);
}

void framefab_gui::SelectPathWidget::buttonForwardUpdateOrderValue()
{
  if((print_order_+1) < max_value_)
  {
    print_order_++;
    orderValueChanged();
  }
}

void framefab_gui::SelectPathWidget::buttonBackwardUpdateOrderValue()
{
  if((print_order_-1) > 0)
  {
    print_order_--;
    orderValueChanged();
  }
}

void framefab_gui::SelectPathWidget::sliderUpdateOrderValue(int value)
{
  print_order_ = value;
  orderValueChanged();
}

void framefab_gui::SelectPathWidget::lineeditUpdateOrderValue(QString value)
{
  print_order_ = value.toInt();
  orderValueChanged();
}