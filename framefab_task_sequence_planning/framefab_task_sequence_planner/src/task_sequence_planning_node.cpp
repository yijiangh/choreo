//
// Created by yijiangh on 12/21/17.
//

#include <ros/ros.h>

// framefab dependencies
#include "framefab_task_sequence_planner/FiberPrintPARM.h"
#include "framefab_task_sequence_planner/utils/WireFrame.h"
#include "framefab_task_sequence_planner/FiberPrintPlugIn.h"

// msg
#include <framefab_msgs/TaskSequencePlanning.h>

// Globals
const static std::string DEFAULT_TASK_SEQUENCE_PLANNING_SERVICE = "task_sequence_planning";

#define SAFE_DELETE_POINTER(ptr) if (ptr!=NULL){delete ptr; ptr = NULL;};


bool planTaskSequenceCallback(framefab_msgs::TaskSequencePlanningRequest& req,
                              framefab_msgs::TaskSequencePlanningResponse& res)
{
  // TODO: add a switch for srv req actions

  double Wl = 1.0;
  double Wp = 1.0;
  double Wa = 1.0;

  // TODO: all of these char* should be const char*
  // convert std::string to writable char*
  std::string& file_path = req.model_params.file_name;
  std::vector<char> fp(file_path.begin(), file_path.end());
  fp.push_back('\0');

  FiberPrintPARM* ptr_parm = new FiberPrintPARM(Wl, Wp, Wa);
  WireFrame* ptr_frame = new WireFrame();

  // TODO: if contains keyword "pwf"
  ptr_frame->LoadFromPWF(&fp[0]);

  FiberPrintPlugIn* ptr_fiberprint =
      new FiberPrintPlugIn(ptr_frame, ptr_parm, &fp[0], true);

  ptr_fiberprint->OneLayerPrint();

  if(ptr_parm != NULL)
  {
    delete ptr_parm;
    ptr_parm = NULL;
  }

  if(ptr_frame != NULL)
  {
    delete ptr_frame;
    ptr_frame = NULL;
  }

  if(ptr_fiberprint != NULL)
  {
    delete ptr_fiberprint;
    ptr_fiberprint = NULL;
  }

  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "framefab_task_sequence_planning");
  ros::NodeHandle nh;

  ros::ServiceServer task_sequence_planning_server =
      nh.advertiseService<framefab_msgs::TaskSequencePlanningRequest, framefab_msgs::TaskSequencePlanningResponse>(
          "task_sequence_planning", boost::bind(planTaskSequenceCallback, _1, _2));

  // Serve and wait for shutdown
  ROS_INFO_STREAM("[tsp] sequence task planning server online");
  ros::spin();

  return 0;
}