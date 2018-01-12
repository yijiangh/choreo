//
// Created by yijiangh on 6/25/17.
//
#include <ros/ros.h>
#include <ros/console.h>

#include <framefab_task_sequence_processor/unit_process.h>
#include <framefab_task_sequence_processor/task_sequence_processor.h>

#include <framefab_rapidjson/include/rapidjson/document.h>
#include <framefab_rapidjson/include/rapidjson/filereadstream.h>

#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

Eigen::Vector3d transformPoint(const Eigen::Vector3d& pt, const double& scale, const Eigen::Vector3d& ref_transf)
{
  return (pt * scale + ref_transf);
}

framefab_task_sequence_processing::TaskSequenceProcessor::TaskSequenceProcessor()
{
  unit_scale_ = 1;
  ref_pt_ = Eigen::Vector3d(0, 0, 0);
  transf_vec_ = Eigen::Vector3d(0, 0, 0);
  verbose_ = false;
}

void framefab_task_sequence_processing::TaskSequenceProcessor::setParams(
    framefab_msgs::ModelInputParameters model_params,
    framefab_msgs::TaskSequenceInputParameters task_sequence_params)
{
  model_input_params_ = model_params;
  path_input_params_ = task_sequence_params;

  // set unit scale
  switch (model_input_params_.unit_type)
  {
    case framefab_msgs::ModelInputParameters::MILLIMETER:
    {
      unit_scale_ = 0.001;
      break;
    }
    case framefab_msgs::ModelInputParameters::CENTIMETER:
    {
      unit_scale_ = 0.01;
      break;
    }
    case framefab_msgs::ModelInputParameters::INCH:
    {
      unit_scale_ = 0.0254;
      break;
    }
    case framefab_msgs::ModelInputParameters::FOOT:
    {
      unit_scale_ = 0.3048;
      break;
    }
    default:
    {
      ROS_ERROR("Unrecognized Unit type in Model Input Parameters!");
    }
  }

  if(verbose_)
  {
    ROS_INFO_STREAM("unit type config succeeded! - unit_scale: " << unit_scale_);
  }

  // element diameter and shrink length (scaled)
  element_diameter_ = model_input_params_.element_diameter * unit_scale_;
  shrink_length_ = model_params.shrink_length * unit_scale_;

  // set ref point
  ref_pt_ = Eigen::Vector3d(model_input_params_.ref_pt_x, model_input_params_.ref_pt_y, model_input_params_.ref_pt_z);
}

bool framefab_task_sequence_processing::TaskSequenceProcessor::createCandidatePoses()
{
  using namespace rapidjson;

  /* --- 1. Parse the input JSON file into a document --- */
  std::string fpath = path_input_params_.file_path;

  FILE* fp = fopen(fpath.c_str(), "r");

  if(NULL == fp)
  {
    ROS_WARN_STREAM("[ts processor] seq result json file not found.");
    return false;
  }

  char readBuffer[65536];
  FileReadStream is(fp, readBuffer, sizeof(readBuffer));

  Document document;

  if(document.ParseStream(is).HasParseError())
  {
    ROS_ERROR_STREAM("TaskSequenceProcessor has ERROR parsing the input json file!");
    return false;
  }

  fclose(fp);

  int m = document["element_number"].GetInt();

  const Value& bcp = document["base_center_pt"];
  Eigen::Vector3d base_center_pt(bcp[0].GetDouble(), bcp[1].GetDouble(), bcp[2].GetDouble());

  if(verbose_)
  {
    ROS_INFO_STREAM("model element member: " << m);
    ROS_INFO_STREAM("base_center_pt: \n" << base_center_pt);
  }

  setTransfVec(ref_pt_, base_center_pt, unit_scale_);

  const Value& process_path_array = document["sequenced_elements"];
  assert(process_path_array.IsArray());

  path_array_.clear();

  for (SizeType i = 0; i < process_path_array.Size(); i++)
  {
    const Value& element_path = process_path_array[i];
    Eigen::Vector3d st_pt(element_path["start_pt"][0].GetDouble(),
                          element_path["start_pt"][1].GetDouble(),
                          element_path["start_pt"][2].GetDouble());

    Eigen::Vector3d end_pt(element_path["end_pt"][0].GetDouble(),
                           element_path["end_pt"][1].GetDouble(),
                           element_path["end_pt"][2].GetDouble());

    std::string type_str = element_path["type"].GetString();

    int wireframe_id = element_path["wireframe_id"].GetInt();

    if(verbose_)
    {
      ROS_INFO_STREAM("element-" << i);
      ROS_INFO_STREAM("start_pt:\n" << st_pt);
      ROS_INFO_STREAM("end_pt:\n" << end_pt);
      ROS_INFO_STREAM("element type - " << type_str);
    }

    // fetch the feasible orients
    std::vector<Eigen::Vector3d> feasible_orients;
    const Value& f_orients = element_path["feasible_orientation"];
    assert(f_orients.IsArray());

    for(SizeType j = 0; j < f_orients.Size(); j++)
    {
      Eigen::Vector3d f_vec(f_orients[j][0].GetDouble(),
                            f_orients[j][1].GetDouble(),
                            f_orients[j][2].GetDouble());
      feasible_orients.push_back(f_vec);

      if (verbose_)
      {
        ROS_INFO_STREAM("feasible orient[" << j << "] =\n" << f_vec);
      }
    }

    // create UnitProcess & Add UnitProcess into ProcessPath
    path_array_.push_back(createScaledUnitProcess(i, wireframe_id, st_pt, end_pt, feasible_orients,
                                                  type_str, element_diameter_, shrink_length_));
  }

  ROS_INFO_STREAM("[task sequence processor] task sequence json parsing succeeded.");
  return true;
}

bool framefab_task_sequence_processing::TaskSequenceProcessor::createEnvCollisionObjs()
{
  // for now, only a simple flat box, representing the build plate, is added.
  // TODO: might need to use load mesh approach for user-customized scene collision setup
  // https://github.com/JeroenDM/descartes_tutorials/blob/indigo-devel/tutorial_utilities/src/collision_object_utils.cpp

  moveit_msgs::CollisionObject collision_env_obj;
  std::string env_obj_id = "env_obj_table";

  // table box's dimension
  double dx = 1;
  double dy = 1;
  double dz = 0.03;

  // pose
  Eigen::Affine3d rtn = Eigen::Translation3d(ref_pt_[0] * unit_scale_,
                                             ref_pt_[1] * unit_scale_,
                                             ref_pt_[2] * unit_scale_ - dz/2)
      * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX())
      * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY())
      * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ());
  geometry_msgs::Pose pose;
  tf::poseEigenToMsg(rtn, pose);

  collision_env_obj.id = env_obj_id;
  collision_env_obj.header.frame_id = "world_frame";
  collision_env_obj.operation = moveit_msgs::CollisionObject::ADD;

  shape_msgs::SolidPrimitive env_obj_solid;
  env_obj_solid.type = shape_msgs::SolidPrimitive::BOX;
  env_obj_solid.dimensions.resize(3);
  env_obj_solid.dimensions[0] = dx;
  env_obj_solid.dimensions[1] = dy;
  env_obj_solid.dimensions[2] = dz;
  collision_env_obj.primitives.push_back(env_obj_solid);
  collision_env_obj.primitive_poses.push_back(pose);

  env_collision_objs_.push_back(collision_env_obj);
  return true;
}

framefab_task_sequence_processing_utils::UnitProcess framefab_task_sequence_processing::TaskSequenceProcessor::createScaledUnitProcess(
    int index, int wireframe_id,
    Eigen::Vector3d st_pt, Eigen::Vector3d end_pt,
    std::vector<Eigen::Vector3d> feasible_orients,
    std::string type_str,
    double element_diameter,
    double shrink_length)
{
  framefab_task_sequence_processing_utils::UnitProcess upp(
      index, wireframe_id,
      transformPoint(st_pt, unit_scale_, transf_vec_),
      transformPoint(end_pt, unit_scale_, transf_vec_),
      feasible_orients, type_str, element_diameter, shrink_length);

  return upp;
}
