//
// Created by yijiangh on 6/27/17.
//

#include <algorithm>

#include <ros/console.h>

#include <QListWidgetItem>
#include <QCheckBox>

#include <ui_selection_widget.h>
#include <framefab_gui/selection/selection_widget.h>

// service
#include <framefab_msgs/ElementNumberRequest.h>
#include <framefab_msgs/VisualizeSelectedPath.h>
#include <framefab_msgs/QueryComputationRecord.h>

const static std::string ELEMENT_NUMBER_REQUEST_SERVICE = "element_member_request";
const static std::string VISUALIZE_SELECTED_PATH = "visualize_select_path";
const static std::string QUERY_COMPUTATION_RESULT = "query_computation_result";

namespace {

// TODO: whenever a new assembly type is added, this part should be extended accordingly.
void convertParsedAssemblyTypeString(const std::string& p_at, framefab_gui::SelectionWidget::ASSEMBLY_TYPE& at)
{
  if("spatial_extrusion" == p_at)
  {
    at = framefab_gui::SelectionWidget::ASSEMBLY_TYPE::SPATIAL_EXTRUSION;
    return;
  }

  if("picknplace" == p_at)
  {
    at = framefab_gui::SelectionWidget::ASSEMBLY_TYPE::PICKNPLACE;
    return;
  }

  // only supports these two for now
  // TODO: default value here for now
  at = framefab_gui::SelectionWidget::ASSEMBLY_TYPE::PICKNPLACE;
//  assert(p_at == "picknplace" || p_at == "spatial_extrusion");
}

} // anon util namespace

framefab_gui::SelectionWidget::SelectionWidget(QWidget* parent) : QWidget(parent),
                                                                  mode_(PATH_SELECTION),
                                                                  sim_type_(SIMULATE_TYPE::SINGLE),
                                                                  selected_value_(0),
                                                                  selected_grasp_id_(0),
                                                                  visualize_ee_(false),
                                                                  use_saved_result_(false)
{
  // UI setup
  ui_ = new Ui::SelectionWidgetWindow;
  ui_->setupUi(this);

  this->setWindowFlags(Qt::WindowStaysOnTopHint);

  select_for_plan_pop_up_ = new SelectForPlanPopUpWidget();
  task_seq_recompute_pop_up_ =  new SelectForPlanPopUpWidget();

  this->select_for_plan_pop_up_->setWindowFlags(Qt::WindowStaysOnTopHint);
  this->task_seq_recompute_pop_up_->setWindowFlags(Qt::WindowStaysOnTopHint);

  // wire in pop up widget signals
  connect(select_for_plan_pop_up_, SIGNAL(buttonRecompute()), this, SLOT(recomputeChosen()));
  connect(select_for_plan_pop_up_, SIGNAL(buttonKeepRecord()), this, SLOT(useSavedResultChosen()));

   // wire in pop up widget signals
  connect(task_seq_recompute_pop_up_, SIGNAL(buttonRecompute()), this, SIGNAL(recomputeTaskSequenceChosen()));
  connect(task_seq_recompute_pop_up_, SIGNAL(buttonKeepRecord()), this, SLOT(useSavedTaskSequenceResultChosen()));
//  connect(task_seq_recompute_pop_up_, SIGNAL(exitSelectForPlanPopUpWidget()), this, SLOT(popUpWindowClosed()));

  // Wire in buttons
  connect(ui_->pushbutton_select_backward, SIGNAL(clicked()), this, SLOT(buttonBackwardUpdateOrderValue()));
  connect(ui_->pushbutton_select_forward, SIGNAL(clicked()), this, SLOT(buttonForwardUpdateOrderValue()));
  connect(ui_->pushbutton_select_for_plan, SIGNAL(clicked()), this, SLOT(buttonSelectForPlan()));

  connect(ui_->pushbutton_select_all, SIGNAL(clicked()), this, SLOT(buttonSelectAll()));

  connect(ui_->pushbutton_simulate_single, SIGNAL(clicked()), this, SLOT(buttonSimulateSingle()));
  connect(ui_->pushbutton_simulate_until, SIGNAL(clicked()), this, SLOT(buttonSimulateUntil()));
  connect(ui_->pushbutton_simulate_chosen, SIGNAL(clicked()), this, SLOT(buttonSimulateChosen()));
  connect(this, SIGNAL(simulateOn(SIMULATE_TYPE)), this, SLOT(buttonSimulate(SIMULATE_TYPE)));

  connect(ui_->pushbutton_clear_chosen, SIGNAL(clicked()), this, SLOT(buttonClearSelection()));
  connect(ui_->pushbutton_set_output_save_dir, SIGNAL(clicked()), this, SIGNAL(setOutputSaveDirOn()));
  connect(ui_->pushbutton_output_generate_chosen, SIGNAL(clicked()), this, SLOT(buttonOutputChosen()));

  connect(ui_->pushbutton_close_widget, SIGNAL(clicked()), this, SLOT(buttonCloseWidget()));

  // Wire in slider
  connect(ui_->slider_select_number, SIGNAL(valueChanged(int)), this, SLOT(sliderUpdateOrderValue(int)));
  connect(ui_->slider_select_grasp, SIGNAL(valueChanged(int)), this, SLOT(sliderUpdateSelectedGraspValue(int)));
  connect(ui_->slider_sim_speed, SIGNAL(valueChanged(int)), this, SLOT(sliderUpdateSimSpeed(int)));

  // Wire in lineedit
  connect(ui_->lineedit_select_number, SIGNAL(returnPressed()), this, SLOT(lineeditUpdateOrderValue()));
  connect(ui_->lineedit_select_grasp, SIGNAL(returnPressed()), this, SLOT(lineeditUpdateSelectedGraspValue()));

  // Wire in checkbox
  connect(ui_->checkbox_visualize_ee, SIGNAL(stateChanged(int)), this, SLOT(checkboxEEVisualUpdateValue()));

  // Start Service Client
  // call core node for visualization request
  visualize_client_ =
      nh_.serviceClient<framefab_msgs::VisualizeSelectedPath>(VISUALIZE_SELECTED_PATH);

  // call core node to fetch element number in (1) parsed assembly seq or (2) computed plans
  query_computation_record_client_ =
      nh_.serviceClient<framefab_msgs::QueryComputationRecord>(QUERY_COMPUTATION_RESULT);
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
      srv.request.action = framefab_msgs::ElementNumberRequest::Request::REQUEST_SELECTED_TASK_NUMBER;
      break;
    }
    default:
    {
      ROS_ERROR_STREAM("Unknown element_number parameter loading request in selection widget");
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

    // fetch back grasp number for each assembly
    grasp_nums_ = srv.response.grasp_nums;

//    ROS_INFO_STREAM("[Selection Widget] select path panel fetch model info successfully.");
  }
  else
  {
    ROS_ERROR_STREAM("[Selection Widget] Unable to fetch model's element number!");
  }

  // reset display value
  selected_value_ = 0;
  ui_->slider_select_number->setValue(0);

  ui_->slider_sim_speed->setValue(1);

  ui_->lineedit_select_number->setText(QString::number(0));

  setInputEnabled(true);

  // trigger update visualization
  orderValueChanged();
}

void framefab_gui::SelectionWidget::setMaxValue(int m)
{
  max_value_ = m - 1;

  ui_->slider_select_number->setMaximum(max_value_);
  ui_->lineedit_select_number->setValidator(new QIntValidator(0, max_value_, this));
  ui_->lineedit_max->setText(QString::number(max_value_));
}

void framefab_gui::SelectionWidget::setMaxGraspNum(int m)
{
  assert(m > 0);
  max_grasp_num_ = m - 1;

  ui_->slider_select_grasp->setMaximum(max_grasp_num_);
  ui_->lineedit_select_grasp->setValidator(new QIntValidator(0, max_grasp_num_, this));
  ui_->lineedit_max_grasp_num->setText(QString::number(max_grasp_num_));
}

void framefab_gui::SelectionWidget::orderValueChanged()
{
  // synchronize assembly seq slider and lineedit
  ui_->slider_select_number->setValue(selected_value_);
  ui_->lineedit_select_number->setText(QString::number(selected_value_));

  // synchronize grasp id slider and lineedit
  assert(grasp_nums_.size() > selected_value_);
  setMaxGraspNum(grasp_nums_[selected_value_]);

  //sync checkbox
  ui_->checkbox_visualize_ee->setChecked(visualize_ee_);


  if(selected_grasp_id_ > max_grasp_num_)
  {
    selected_grasp_id_ = 0;
  }

  // synchronize grasp selection slider and lineedit
  ui_->slider_select_grasp->setValue(selected_grasp_id_);
  ui_->lineedit_select_grasp->setText(QString::number(selected_grasp_id_));

  // call visualization srv
  framefab_msgs::VisualizeSelectedPath srv;

  // TODO: the assembly task type should be a part of model param
  // hardcoded to picknplace for now
  srv.request.assembly_type = srv.request.PICKNPLACE;
  srv.request.index = selected_value_;
  srv.request.visualize_ee = visualize_ee_;
  srv.request.grasp_id = selected_grasp_id_;

//  if(PATH_SELECTION == mode_)
//  {
//
//  }

  if(PLAN_SELECTION == mode_)
  {
    ui_->plan_list_widget->clearSelection();

    std::vector<int> not_found_index;
    std::vector<int>::iterator it;

    // update list widget selection, all until selected_value_
    for(std::size_t i=0; i <= selected_value_; i++)
    {
      std::vector<int>::iterator it =
          std::find(fetched_plan_ids_.begin(), fetched_plan_ids_.end(), i);

      if(it != fetched_plan_ids_.end())
      {
        ui_->plan_list_widget->item(it - fetched_plan_ids_.begin())->setSelected(true);
      }
      else
      {
        not_found_index.push_back(i);
      }
    }

    if(0 != not_found_index.size())
    {
      std::string error_msg = std::to_string(not_found_index.size()) + " plans not found";
      ui_->status_bar->setStyleSheet("QLabel { color : red; }");
      ui_->status_bar->setText(QString::fromStdString(error_msg));
    }
  }

  setInputEnabled(false);

  visualize_client_.waitForExistence();
  if (!visualize_client_.call(srv))
  {
    ROS_ERROR_STREAM("UI: Unable to visualize selected path!!");
  }

  setInputEnabled(true);
}

void framefab_gui::SelectionWidget::simSpeedChanged()
{
  ui_->lineedit_sim_speed->setText(QString::number(sim_speed_));
}

static int getIntFromString(const std::string &str)
{
  std::string::size_type sz;   // alias of size_t

  int i_dec = std::stoi(str, &sz);

  return i_dec;
}

void framefab_gui::SelectionWidget::addFetchedPlans(const std::vector<std::string> &plan_names)
{
  ui_->plan_list_widget->clear();
  fetched_plan_ids_.clear();

  for (const auto& plan : plan_names)
  {
    // add it in fetched_plans for available plan database
    fetched_plan_ids_.push_back(getIntFromString(plan));
  }

  // sort fetched plans in an increasing order
  std::sort(fetched_plan_ids_.begin(), fetched_plan_ids_.end());

  for (const auto& id : fetched_plan_ids_)
  {
    QListWidgetItem *item = new QListWidgetItem();
    item->setText(QString::number(id));
    ui_->plan_list_widget->addItem(item);
  }
}

void framefab_gui::SelectionWidget::getChosenPlans()
{
  chosen_ids_for_sim_.clear();

  QList<QListWidgetItem*> qt_chosen_items = ui_->plan_list_widget->selectedItems();

  for(auto q_item : qt_chosen_items)
  {
//    ROS_INFO_STREAM("chosen: " << q_item->text().toStdString());
    chosen_ids_for_sim_.push_back(getIntFromString(q_item->text().toStdString()));
  }

  // sort in increasing index order
  std::sort(chosen_ids_for_sim_.begin(), chosen_ids_for_sim_.end());
}

void framefab_gui::SelectionWidget::setStatusBar(std::string string, bool state)
{
  if(state)
  {
    ui_->status_bar->setStyleSheet("QLabel { color : green; }");
  }
  else
  {
    ui_->status_bar->setStyleSheet("QLabel { color : red; }");
  }

  ui_->status_bar->setText(QString::fromStdString(string));
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

void framefab_gui::SelectionWidget::setAssemblyType(const std::string& at)
{
  convertParsedAssemblyTypeString(at, assembly_type_);
}

void framefab_gui::SelectionWidget::showTaskSequenceRecomputePopUp(bool found_task_plan)
{
  if(SPATIAL_EXTRUSION == assembly_type_)
  {
    if (found_task_plan)
    {
      std::string msg = "Saved task sequence plan record found.";
      task_seq_recompute_pop_up_->setDisplayText(msg);
    }
    else
    {
      ROS_WARN_STREAM("[UI] No saved task sequence plan found.");

      std::string msg = "No saved task sequence plan record found.";
      task_seq_recompute_pop_up_->setDisplayText(msg);
    }

    task_seq_recompute_pop_up_->enableButtons(found_task_plan);
  }

  // TODO: currently we don't support task sequence planning for picknplace with arbitrary geometry
  // This should be removed later
  if(PICKNPLACE == assembly_type_)
  {
    std::string msg = "Sorry... Currently we don't support sequence planning for general picknplace.";
    task_seq_recompute_pop_up_->setDisplayText(msg);

    task_seq_recompute_pop_up_->enableButtons(found_task_plan, false);
  }

  task_seq_recompute_pop_up_->show();
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
  if(mode_ == PATH_SELECTION)
  {
    ui_->pushbutton_select_backward->setEnabled(enabled);
    ui_->pushbutton_select_forward->setEnabled(enabled);
    ui_->slider_select_number->setEnabled(enabled);
    ui_->lineedit_select_number->setEnabled(enabled);

    ui_->pushbutton_select_for_plan->setEnabled(enabled);
    ui_->pushbutton_select_all->setEnabled(enabled);
    ui_->pushbutton_simulate_single->setEnabled(false);
    ui_->pushbutton_simulate_until->setEnabled(false);

    setInputEndEffectorVisualEnabled(enabled);
    setInputGraspEnabled(enabled);

    // TODO: to be implemented
    setInputIKSolutionEnabled(false);

    ui_->pushbutton_close_widget->setEnabled(false);

    // set tab_widget to <grasp visualizer> and disable <traj lib> tab
    ui_->tab_widget->setEnabled(enabled);
    ui_->tab_widget->setTabEnabled(0, enabled);
    ui_->tab_widget->setTabEnabled(1, false);
    ui_->tab_widget->setCurrentIndex(0);
  }

  if(mode_ == ZOOM_IN_SELECTION)
  {
    enabled = false;

    ui_->pushbutton_select_backward->setEnabled(enabled);
    ui_->pushbutton_select_forward->setEnabled(enabled);
    ui_->slider_select_number->setEnabled(enabled);
    ui_->lineedit_select_number->setEnabled(enabled);

    ui_->pushbutton_select_for_plan->setEnabled(false);
    ui_->pushbutton_select_all->setEnabled(false);
    ui_->pushbutton_simulate_single->setEnabled(false);
    ui_->pushbutton_simulate_until->setEnabled(false);

    setInputEndEffectorVisualEnabled(enabled);
    setInputGraspEnabled(enabled);
    setInputIKSolutionEnabled(enabled);

    ui_->pushbutton_close_widget->setEnabled(false);

    ui_->tab_widget->setEnabled(enabled);
    ui_->tab_widget->setTabEnabled(0, enabled);
    ui_->tab_widget->setTabEnabled(1, false);
    ui_->tab_widget->setCurrentIndex(0);
  }

  if(mode_ == PLAN_SELECTION)
  {
    ui_->pushbutton_select_backward->setEnabled(enabled);
    ui_->pushbutton_select_forward->setEnabled(enabled);
    ui_->slider_select_number->setEnabled(enabled);
    ui_->lineedit_select_number->setEnabled(enabled);

    ui_->pushbutton_select_for_plan->setEnabled(false);
    ui_->pushbutton_select_all->setEnabled(enabled);
    ui_->pushbutton_simulate_single->setEnabled(enabled);
    ui_->pushbutton_simulate_until->setEnabled(enabled);

    setInputGraspEnabled(false);
    setInputIKSolutionEnabled(false);

    ui_->pushbutton_close_widget->setEnabled(true);

    ui_->tab_widget->setEnabled(enabled);
    ui_->tab_widget->setTabEnabled(0, false);
    ui_->tab_widget->setTabEnabled(1, true);
    ui_->tab_widget->setCurrentIndex(1);
  }
}

void framefab_gui::SelectionWidget::setInputEndEffectorVisualEnabled(bool enabled)
{
  ui_->checkbox_visualize_ee->setEnabled(enabled);
}

void framefab_gui::SelectionWidget::setInputGraspEnabled(bool enabled)
{
  ui_->slider_select_grasp->setEnabled(enabled);
  ui_->lineedit_select_grasp->setEnabled(enabled);
}

void framefab_gui::SelectionWidget::setInputIKSolutionEnabled(bool enabled)
{
  ui_->slider_select_ik_solution->setEnabled(enabled);
  ui_->lineedit_select_ik_solution->setEnabled(enabled);
}

void framefab_gui::SelectionWidget::buttonForwardUpdateOrderValue()
{
  if((selected_value_+1) <= max_value_)
  {
    selected_value_++;
    orderValueChanged();
  }
}

void framefab_gui::SelectionWidget::buttonBackwardUpdateOrderValue()
{
  if((selected_value_-1) >= 0)
  {
    selected_value_--;
    orderValueChanged();
  }
}

void framefab_gui::SelectionWidget::buttonSelectAll()
{
  selected_value_ = max_value_;
  orderValueChanged();

//  Q_EMIT acceptSelection();
}

void framefab_gui::SelectionWidget::buttonSimulate(SIMULATE_TYPE sim_type)
{
  // update internal simulate type
  sim_type_ = sim_type;
  selected_ids_for_sim_.clear();

  // update selected_ids for simulation
  switch (sim_type_)
  {
    case SIMULATE_TYPE::SINGLE:
    {
      selected_ids_for_sim_.push_back(selected_value_);


      break;
    }
    case SIMULATE_TYPE::ALL_UNTIL:
    {
      for(int i = 0; i <= selected_value_; i++)
      {
        selected_ids_for_sim_.push_back(i);
      }

      break;
    }
    case SIMULATE_TYPE::CHOSEN:
    {
      getChosenPlans();
      selected_ids_for_sim_ = chosen_ids_for_sim_;

      if(0 == selected_ids_for_sim_.size())
      {
        ROS_WARN("No ids is chosen!");
        return;
      }

      selected_ids_for_sim_ = chosen_ids_for_sim_;

      break;
    }
  }

  // wait for simulation to be completed
  setInputEnabled(false);
  Q_EMIT flushSimulation();
}

void framefab_gui::SelectionWidget::buttonOutputChosen()
{
  getChosenPlans();
  Q_EMIT flushOutputProcess();
}

void framefab_gui::SelectionWidget::buttonClearSelection()
{
  ui_->plan_list_widget->clearSelection();
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

  framefab_msgs::QueryComputationRecord srv;
  bool saved_record_found = false;

  srv.request.action = framefab_msgs::QueryComputationRecordRequest::SAVED_LADDER_GRAPH;
  srv.request.file_name = model_file_name_;

  query_computation_record_client_.waitForExistence();

  if (query_computation_record_client_.call(srv))
  {
//    ROS_INFO_STREAM("[Selection Widget] select path panel fetch saved ladder graph info successfully.");

    saved_record_found = srv.response.record_found;
    const int found_record_size = srv.response.found_record_size;

    // set pop up widget text
    if(saved_record_found && found_record_size > 0)
    {
      std::string msg = "Previous ladder graph record found: 0 - " + std::to_string(found_record_size-1);
      select_for_plan_pop_up_->setDisplayText(msg);
    }
    else
    {
      ROS_WARN_STREAM("[UI] No previous ladder graph record found.");

      std::string msg = "No previous ladder graph record found.";
      select_for_plan_pop_up_->setDisplayText(msg);
    }
  }
  else
  {
    std::string msg = "Unable to fetch model's element number.";
    select_for_plan_pop_up_->setDisplayText(msg);
    saved_record_found = false;
  }

  select_for_plan_pop_up_->enableButtons(saved_record_found);
  select_for_plan_pop_up_->show();

  setMode(ZOOM_IN_SELECTION);
  setInputEnabled(true);
}

void framefab_gui::SelectionWidget::sliderUpdateOrderValue(int value)
{
  selected_value_ = value;
  orderValueChanged();
}

void framefab_gui::SelectionWidget::lineeditUpdateOrderValue()
{
  selected_value_ = ui_->lineedit_select_number->text().toInt();
  orderValueChanged();
}

void framefab_gui::SelectionWidget::checkboxEEVisualUpdateValue()
{
  visualize_ee_ = ui_->checkbox_visualize_ee->isChecked();
  orderValueChanged();
}

void framefab_gui::SelectionWidget::sliderUpdateSelectedGraspValue(int value)
{
  selected_grasp_id_ = value;
  orderValueChanged();
}

void framefab_gui::SelectionWidget::lineeditUpdateSelectedGraspValue()
{
  selected_value_ = ui_->lineedit_select_grasp->text().toInt();
  orderValueChanged();
}

void framefab_gui::SelectionWidget::sliderUpdateSimSpeed(int value)
{
  // TODO: finish this
  // discretization of slider is set in qt ui file
  sim_speed_ = (double) value / 100;
  simSpeedChanged();
}

void framefab_gui::SelectionWidget::recomputeChosen()
{
  use_saved_result_ = false;
  this->close();

  Q_EMIT closeWidgetAndContinue();
}

void framefab_gui::SelectionWidget::useSavedResultChosen()
{
  use_saved_result_ = true;
  this->close();

  Q_EMIT closeWidgetAndContinue();
}

void framefab_gui::SelectionWidget::useSavedTaskSequenceResultChosen()
{
  Q_EMIT closeWidgetAndContinue();
}

void framefab_gui::SelectionWidget::popUpWindowClosed()
{
  Q_EMIT enableFrameFabWidgetButtons();
}