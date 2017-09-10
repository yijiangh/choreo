//
// Created by yijiangh on 6/27/17.
//

#include <ros/console.h>

#include <ui_selection_widget.h>
#include <framefab_gui/selection/selection_widget.h>

// service
#include <framefab_msgs/ElementNumberRequest.h>
#include <framefab_msgs/VisualizeSelectedPath.h>

const static std::string ELEMENT_NUMBER_REQUEST_SERVICE = "element_member_request";
const static std::string VISUALIZE_SELECTED_PATH = "visualize_select_path";

framefab_gui::SelectionWidget::SelectionWidget(QWidget* parent) : QWidget(parent),
                                                                  mode_(PATH_SELECTION), simulate_single_(true)
{
  // UI setup
  ui_ = new Ui::SelectionWidgetWindow;
  ui_->setupUi(this);

  this->setWindowFlags(Qt::WindowStaysOnTopHint);

  // Wire in buttons
  connect(ui_->pushbutton_select_backward, SIGNAL(clicked()), this, SLOT(buttonBackwardUpdateOrderValue()));
  connect(ui_->pushbutton_select_forward, SIGNAL(clicked()), this, SLOT(buttonForwardUpdateOrderValue()));
  connect(ui_->pushbutton_select_for_plan, SIGNAL(clicked()), this, SLOT(buttonSelectForPlan()));

  connect(ui_->pushbutton_simulate_single, SIGNAL(clicked()), this, SLOT(buttonSimulateSingle()));
  connect(ui_->pushbutton_simulate_until, SIGNAL(clicked()), this, SLOT(buttonSimulateUntil()));
  connect(this, SIGNAL(simulateOn(SIMULATE_TYPE)), this, SLOT(buttonSimulate(SIMULATE_TYPE)));

  connect(ui_->pushbutton_select_all, SIGNAL(clicked()), this, SLOT(buttonSelectAll()));

  connect(ui_->pushbutton_close_widget, SIGNAL(clicked()), this, SLOT(buttonCloseWidget()));

  // Wire in slider
  connect(ui_->slider_select_number, SIGNAL(valueChanged(int)), this, SLOT(sliderUpdateOrderValue(int)));

  // Wire in lineedit
  connect(ui_->lineedit_select_number, SIGNAL(returnPressed()), this, SLOT(lineeditUpdateOrderValue()));

  // Start Service Client
  visualize_client_ =
      nh_.serviceClient<framefab_msgs::VisualizeSelectedPath>(VISUALIZE_SELECTED_PATH);
}

void framefab_gui::SelectionWidget::loadParameters()
{
  framefab_msgs::ElementNumberRequest srv;

  switch (mode_)
  {
    case PATH_SELECTION:
    {
      srv.request.action = framefab_msgs::ElementNumberRequest::Request::REQUEST_ELEMENT_NUMBER;
      break;
    }
    case PLAN_SELECTION:
    {
      srv.request.action = framefab_msgs::ElementNumberRequest::Request::REQUEST_SELECTED_PATH_NUMBER;
      break;
    }
    default:
    {
      ROS_ERROR_STREAM("Unknown parameter loading request in selection widget");
      break;
    }
  }

  ros::ServiceClient element_number_param_client =
      nh_.serviceClient<framefab_msgs::ElementNumberRequest>(ELEMENT_NUMBER_REQUEST_SERVICE);

  setInputEnabled(false);

  element_number_param_client.waitForExistence();

  if (element_number_param_client.call(srv))
  {
    this->setMaxValue(srv.response.element_number);

//    ROS_INFO_STREAM("[Selection Widget] select path panel fetch model info successfully.");
  }
  else
  {
    ROS_ERROR_STREAM("[Selection Widget] Unable to fetch model's element number!");
  }

  // reset display value
  print_order_ = 0;
  ui_->slider_select_number->setValue(0);
  ui_->lineedit_select_number->setText(QString::number(0));

  setInputEnabled(true);

  // update visualization
  orderValueChanged();
}

void framefab_gui::SelectionWidget::setMaxValue(int m)
{
  if(mode_ == PATH_SELECTION)
  {
    max_value_ = m - 1;
  }

  if(mode_ == PLAN_SELECTION)
  {
    max_value_ = m;
  }

  ui_->slider_select_number->setMaximum(max_value_);
  ui_->lineedit_select_number->setValidator(new QIntValidator(0, max_value_, this));
  ui_->lineedit_max->setText(QString::number(max_value_));
}

void framefab_gui::SelectionWidget::orderValueChanged()
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

void framefab_gui::SelectionWidget::cleanUpVisual()
{
 // call visualization srv
  framefab_msgs::VisualizeSelectedPath srv;
  srv.request.index = -1;

  visualize_client_.waitForExistence();
  if (!visualize_client_.call(srv))
  {
    ROS_ERROR_STREAM("Unable to clean up selected path!!");
  }
}

void framefab_gui::SelectionWidget::showEvent(QShowEvent *ev)
{
  Q_EMIT enterSelectionWidget();
}

void framefab_gui::SelectionWidget::closeEvent(QCloseEvent *ev)
{
  Q_EMIT exitSelectionWidget();
}

void framefab_gui::SelectionWidget::setInputEnabled(bool enabled)
{
  ui_->pushbutton_select_backward->setEnabled(enabled);
  ui_->pushbutton_select_forward->setEnabled(enabled);

  if(mode_ == PATH_SELECTION)
  {
    ui_->pushbutton_select_for_plan->setEnabled(enabled);
    ui_->pushbutton_select_all->setEnabled(enabled);
    ui_->pushbutton_simulate_single->setEnabled(false);
    ui_->pushbutton_simulate_until->setEnabled(false);

    setInputIDEnabled(false);
    setInputLocaAxisEnabled(false);
    setInputIKSolutionEnabled(false);

    ui_->pushbutton_simulate_single_process->setEnabled(false);
    ui_->pushbutton_close_widget->setEnabled(false);

    ui_->tab_widget->setEnabled(false);
  }

  if(mode_ == ZOOM_IN_SELECTION)
  {
    ui_->pushbutton_select_for_plan->setEnabled(false);
    ui_->pushbutton_select_all->setEnabled(false);
    ui_->pushbutton_simulate_single->setEnabled(false);
    ui_->pushbutton_simulate_until->setEnabled(false);

    setInputIDEnabled(enabled);
    setInputLocaAxisEnabled(enabled);
    setInputIKSolutionEnabled(enabled);

    ui_->pushbutton_simulate_single_process->setEnabled(enabled);
    ui_->pushbutton_close_widget->setEnabled(enabled);

    ui_->tab_widget->setEnabled(true);
    ui_->tab_widget->setTabEnabled(0, true);
    ui_->tab_widget->setTabEnabled(1, false);
    ui_->tab_widget->setCurrentIndex(0);
  }

  if(mode_ == PLAN_SELECTION)
  {
    ui_->pushbutton_select_for_plan->setEnabled(false);
    ui_->pushbutton_select_all->setEnabled(enabled);
    ui_->pushbutton_simulate_single->setEnabled(enabled);
    ui_->pushbutton_simulate_until->setEnabled(enabled);

    setInputIDEnabled(false);
    setInputLocaAxisEnabled(false);
    setInputIKSolutionEnabled(false);

    ui_->pushbutton_simulate_single_process->setEnabled(false);
    ui_->pushbutton_close_widget->setEnabled(false);

    ui_->tab_widget->setEnabled(true);
    ui_->tab_widget->setTabEnabled(0, false);
    ui_->tab_widget->setTabEnabled(1, true);
    ui_->tab_widget->setCurrentIndex(1);
  }

  ui_->slider_select_number->setEnabled(enabled);
  ui_->lineedit_select_number->setEnabled(enabled);
}

void framefab_gui::SelectionWidget::setInputIDEnabled(bool enabled)
{
  ui_->slider_select_orient->setEnabled(enabled);
  ui_->lineedit_select_orient->setEnabled(enabled);
}

void framefab_gui::SelectionWidget::setInputLocaAxisEnabled(bool enabled)
{
  ui_->slider_select_local_axis->setEnabled(enabled);
  ui_->lineedit_select_local_axis->setEnabled(enabled);
}

void framefab_gui::SelectionWidget::setInputIKSolutionEnabled(bool enabled)
{
  ui_->slider_select_ik_solution->setEnabled(enabled);
  ui_->lineedit_select_ik_solution->setEnabled(enabled);
}

void framefab_gui::SelectionWidget::buttonForwardUpdateOrderValue()
{
  if((print_order_+1) <= max_value_)
  {
    print_order_++;
    orderValueChanged();
  }
}

void framefab_gui::SelectionWidget::buttonBackwardUpdateOrderValue()
{
  if((print_order_-1) >= 0)
  {
    print_order_--;
    orderValueChanged();
  }
}

void framefab_gui::SelectionWidget::buttonSelectAll()
{
  print_order_ = max_value_;
  orderValueChanged();
  simulate_single_ = false;

//  Q_EMIT acceptSelection();
}

void framefab_gui::SelectionWidget::buttonSimulate(SIMULATE_TYPE sim_type)
{
//  simulate_single_ = single;
  ROS_INFO("simulate button pressed!");
}

void framefab_gui::SelectionWidget::buttonSimulateSingle()
{
  Q_EMIT simulateOn(SIMULATE_TYPE::SINGLE);
}

void framefab_gui::SelectionWidget::buttonSimulateUntil()
{
  Q_EMIT simulateOn(SIMULATE_TYPE::ALL_UNTIL);
}

void framefab_gui::SelectionWidget::buttonSimulateChosen()
{
  Q_EMIT simulateOn(SIMULATE_TYPE::CHOSEN);
}

void framefab_gui::SelectionWidget::buttonCloseWidget()
{
  this->close();
}

void framefab_gui::SelectionWidget::buttonSelectForPlan()
{
  setInputEnabled(false);

  // send service to core to enable pre-graph construction
//  Q_EMIT acceptSelection();

  setMode(ZOOM_IN_SELECTION);
  setInputEnabled(true);
}

void framefab_gui::SelectionWidget::sliderUpdateOrderValue(int value)
{
  print_order_ = value;
  orderValueChanged();
}

void framefab_gui::SelectionWidget::lineeditUpdateOrderValue()
{
  print_order_ = ui_->lineedit_select_number->text().toInt();
  orderValueChanged();
}
