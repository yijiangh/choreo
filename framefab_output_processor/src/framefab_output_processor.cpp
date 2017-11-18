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

namespace
{
std::string translateProcessType(const int& process_type_id)
{
  // ref framefab_msgs/SubProcess.msg process_type table
  switch(process_type_id)
  {
    case 0:
    {
      return std::string("Process");
    }
    case 1:
    {
      return std::string("Near Model");
    }
    case 2:
    {
      return std::string("Transition");
    }
    default:
    {
      ROS_WARN("[output processor] Unknown process_type. Save default 'None' type.");
      return std::string("Unknown");
    }
  }
}

std::string translateMainDataType(const int& main_data_type_id)
{
  // ref framefab_msgs/SubProcess.msg main_data_type_type table
  switch(main_data_type_id)
  {
    case -1:
    {
      return std::string("Joint");
    }
    case -2:
    {
      return std::string("Cartesian");
    }
    default:
    {
      ROS_ERROR("[output processor] Unknown process_type. Save default 'Joint' type.");
      return std::string("Joint");
    }
  }
}

std::string translateElementProcessType(const int& element_process_type_id)
{
  // ref framefab_msgs/SubProcess.msg element_process_type table
  switch(element_process_type_id)
  {
    case 3:
    {
      return std::string("Support");
    }
    case 4:
    {
      return std::string("Create");
    }
    case 5:
    {
      return std::string("Connect");
    }
    case 6:
    {
      return std::string("None");
    }
    default:
    {
      ROS_WARN("[output processor] Unknown element_process_type. Save default 'None' type");
      return std::string("None");
    }
  }
}
}// name space util


bool framefab_output_processor::OutputProcessor::outputJson(std::vector<framefab_msgs::UnitProcessPlan>& plans)
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

  for(const auto& unit_process_plan : plans)
  {
    rapidjson::Value unit_process_plan_container(rapidjson::kArrayType);

    for (const auto &sub_process : unit_process_plan.sub_process_array)
    {
      rapidjson::Value sub_process_object_container(rapidjson::kObjectType);

      // parent_unit_process_plan_id
      sub_process_object_container.AddMember("parent_unit_process_plan_id", sub_process.unit_process_id, allocator);

      // subprocess_id
      sub_process_object_container.AddMember("sub_process_id", sub_process.sub_process_id, allocator);

      // process_type
      sub_process_object_container.AddMember("process_type",
                                             Value().SetString(translateProcessType(sub_process.process_type).c_str(), allocator),
                                             allocator);

      // main_data_type
      sub_process_object_container.AddMember("main_data_type",
                                             Value().SetString(translateMainDataType(sub_process.main_data_type).c_str(), allocator),
                                             allocator);

      // joint_array
      rapidjson::Value joint_array_container(rapidjson::kArrayType);

      for(const auto& joint_pt : sub_process.joint_array.points)
      {
        rapidjson::Value joint_pt_container(rapidjson::kArrayType);

        // add robot joint values
        for (const auto& joint_value : joint_pt.positions)
        {
          joint_pt_container.PushBack(Value().SetDouble(joint_value), allocator);
        }

        joint_array_container.PushBack(joint_pt_container, allocator);
      }

      sub_process_object_container.AddMember("joint_array", joint_array_container, allocator);

      // TCP_pose_array
      rapidjson::Value TCP_pose_array_container(rapidjson::kArrayType);

      for(const auto& TCP_pose_pt : sub_process.TCP_pose_array)
      {
        // add robot joint values
        rapidjson::Value TCP_pose_pt_container(rapidjson::kObjectType);

        // from geometry pose to affine3d matrix
        Eigen::Affine3d m;
        tf::poseMsgToEigen(TCP_pose_pt, m);
        Eigen::Matrix3d orientation = m.matrix().block<3,3>(0,0);
        Eigen::Vector3d position = m.matrix().col(3).head<3>();

        // add axes
        for(int i=0; i < 3; i++)
        {
          rapidjson::Value axis_vector3d_container(rapidjson::kArrayType);
          // x, y, z axis
          for (int j = 0; j < 3; j++)
          {
            // coordinates for axis
            axis_vector3d_container.PushBack(Value().SetDouble(orientation.col(i)[j]), allocator);
          }

          std::string axis_name = "axis_" + std::to_string(i);
          TCP_pose_pt_container.AddMember(Value().SetString(axis_name.c_str(), allocator), axis_vector3d_container, allocator);
        }

        // add origin position
       rapidjson::Value TCP_origin_container(rapidjson::kArrayType);
        for (int j = 0; j < 3; j++)
        {
          TCP_origin_container.PushBack(Value().SetDouble(position[j]), allocator);
        }
        TCP_pose_pt_container.AddMember("origin", TCP_origin_container, allocator);

        // add TCP pose into TCP_array
        TCP_pose_array_container.PushBack(TCP_pose_pt_container, allocator);
      }

      sub_process_object_container.AddMember("TCP_pose_array", TCP_pose_array_container, allocator);

      // comment or process explanation (connect, create, support etc.)
      // TODO

      unit_process_plan_container.PushBack(sub_process_object_container, allocator);
    } // end subprocess

    process_plans_container.PushBack(unit_process_plan_container, allocator);
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