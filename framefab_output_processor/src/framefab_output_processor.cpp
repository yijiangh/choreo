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
      sub_process_object_container.AddMember("process_type", sub_process.process_type, allocator);

      // main_data_type
      sub_process_object_container.AddMember("main_data_type", sub_process.main_data_type, allocator);

      // joint_array
      rapidjson::Value traj_joint_array_containter(rapidjson::kArrayType);
      traj_joint_array_containter.Clear();
      rapidjson::Value traj_joint_pt_container(rapidjson::kArrayType);

      for(const auto& traj_joint_pt : sub_process.joint_array.points)
      {
        // add robot joint values
        traj_joint_pt_container.Clear();

        for (const auto& joint_value : traj_joint_pt.positions)
        {
          traj_joint_pt_container.PushBack(Value().SetDouble(joint_value), allocator);
        }

        traj_joint_array_containter.PushBack(traj_joint_pt_container, allocator);
      }

      sub_process_object_container.AddMember("joint_array", traj_joint_array_containter, allocator);

      // TCP_pose_array
      rapidjson::Value TCP_pose_array_containter(rapidjson::kArrayType);
      TCP_pose_array_containter.Clear();

      for(const auto& TCP_pose_pt : sub_process.TCP_pose_array)
      {
        // add robot joint values
        rapidjson::Value TCP_pose_pt_container(rapidjson::kObjectType);

        // from geometry pose to affine3d matrix
        Eigen::Affine3d m;
        tf::poseMsgToEigen(TCP_pose_pt, m);
        Eigen::Matrix3d orientation = m.matrix().block<3,3>(0,0);
        Eigen::Vector3d position = m.matrix().col(3).head<3>();

        rapidjson::Value vector3d_container(rapidjson::kArrayType);

        // add axes
        for(int i=0; i < 2; i++)
        {
          // x, y, z axis
          vector3d_container.Clear();

          for (int j = 0; j < 2; j++)
          {
            // coordinates for axis
            vector3d_container.PushBack(Value().SetDouble(orientation.col(i)[j]), allocator);
          }

          std::string axis_name = "axis_" + std::to_string(i);
          TCP_pose_pt_container.AddMember(Value().SetString(axis_name.c_str(), allocator), vector3d_container, allocator);
        }

        // add origin position
        vector3d_container.Clear();
        for (int j = 0; j < 2; j++)
        {
          vector3d_container.PushBack(Value().SetDouble(position[j]), allocator);
        }
        TCP_pose_pt_container.AddMember("origin", vector3d_container, allocator);

        // add TCP pose into TCP_array
        TCP_pose_array_containter.PushBack(TCP_pose_pt_container, allocator);
      }

      sub_process_object_container.AddMember("TCP_pose_array", TCP_pose_array_containter, allocator);

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