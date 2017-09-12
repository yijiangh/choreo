//
// Created by yijiangh on 9/10/17.
//

#include <ros/ros.h>
#include <ros/console.h>

#include <framefab_output_processor/framefab_output_processor.h>

#include <framefab_rapidjson/include/rapidjson/document.h>
#include <framefab_rapidjson/include/rapidjson/filewritestream.h>
#include <framefab_rapidjson/include/rapidjson/prettywriter.h>

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

  Value model_object_container(rapidjson::kArrayType);
  document.AddMember("process_num", plans.size(), allocator);
//  document.AddMember("support_num", support_, allocator);

  for (int i = 0; i < plans.size(); i++)
  {
    rapidjson::Value unit_process_container(rapidjson::kObjectType);

    // sub_process
    // id: sub_process i
    // main_data_type: "cartesian" or "joints"
    // trajectory_points: [array]
    // EEF_plans: [array]
    // comment:

    // temporal testing, only "process" process type
    rapidjson::Value sub_process_object_container(rapidjson::kObjectType);
    sub_process_object_container.AddMember("process type", "process", allocator);

    rapidjson::Value trajectory_points(rapidjson::kArrayType);
    trajectory_points.Clear();

    for(auto traj : plans[i].trajectory_process.points)
    {
      rapidjson::Value joint_traj_point(rapidjson::kArrayType);
      for(auto joint_value : )
      feasible_orient.PushBack(Value().SetDouble(truncDigits(temp.normal_[j].getX(), FF_TRUNC_SCALE)), allocator);

    }

  }

  return true;
}