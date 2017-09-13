//
// Created by yijiangh on 9/10/17.
//

#include <ros/ros.h>
#include <ros/console.h>

#include <framefab_output_processor/framefab_output_processor.h>

#include <framefab_rapidjson/include/rapidjson/document.h>
#include <framefab_rapidjson/include/rapidjson/filewritestream.h>
#include <framefab_rapidjson/include/rapidjson/prettywriter.h>

//msg
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

bool framefab_output_processor::OutputProcessor::outputJson(std::vector<framefab_msgs::UnitProcessPlan> &plans)
{
  using namespace rapidjson;

  // document is the root of a json message
  rapidjson::Document document;

  // define the document as an object rather than an array
  document.SetObject();

  // must pass an allocator when the object may need to allocate memory
  rapidjson::Document::AllocatorType& allocator = document.GetAllocator();

  document.AddMember("process_num", plans.size(), allocator);
//  document.AddMember("support_num", support_, allocator);

  Value process_plans_container(rapidjson::kArrayType);

  for (int i = 0; i < plans.size(); i++)
  {
    rapidjson::Value unit_process_container(rapidjson::kArrayType);

    // sub_process
    // id: sub_process i
    // main_data_type: "cartesian" or "joints"
    // trajectory_points: [array]
    // EEF_plans: [array]
    // comment:

    // TODO: loop for subprocesses
//    {
    // temporal testing, only "process" process type
    rapidjson::Value sub_process_object_container(rapidjson::kObjectType);
    sub_process_object_container.AddMember("process type", "process", allocator);

    rapidjson::Value trajectory_points(rapidjson::kArrayType);
    trajectory_points.Clear();

    // add every traj in the subprocess
    for (trajectory_msgs::JointTrajectoryPoint traj_pt : plans[i].trajectory_process.points)
    {
      // add robot joint values
      rapidjson::Value joint_traj_point(rapidjson::kArrayType);
      for (auto joint_value : traj_pt.positions)
      {
        joint_traj_point.PushBack(Value().SetDouble(joint_value), allocator);
      }

      trajectory_points.PushBack(joint_traj_point, allocator);
    }

    sub_process_object_container.AddMember("trajectory_points", trajectory_points, allocator);

    // add more for subprocess

    unit_process_container.PushBack(sub_process_object_container, allocator);
//    }


    process_plans_container.PushBack(unit_process_container, allocator);
  }

  document.AddMember("process_plans", process_plans_container, allocator);

  // output files to path
  FILE *js_file = fopen(this->save_file_path_.c_str(), "w+");
  if(NULL == js_file)
  {
    ROS_ERROR("[output_processor]: invalid output file path!!!");
  }

  char writeBuffer[65536];
  FileWriteStream os(js_file, writeBuffer, sizeof(writeBuffer));

  PrettyWriter<FileWriteStream> p_writer(os);
  document.Accept(p_writer);

  std::fclose(js_file);
  ROS_INFO("[output_processor] json output file saved successfully!");

  return true;
}