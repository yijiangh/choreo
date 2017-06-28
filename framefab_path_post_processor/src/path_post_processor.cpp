//
// Created by yijiangh on 6/25/17.
//
#include <ros/ros.h>
#include <ros/console.h>

#include <framefab_path_post_processor/path_post_processor.h>

#include <rapidjson/document.h>
#include "rapidjson/filereadstream.h"


Eigen::Vector3d transformPoint(const Eigen::Vector3d& pt, const double& scale, const Eigen::Vector3d& ref_transf)
{
  return (pt * scale + ref_transf);
}

framefab_path_post_processing::PathPostProcessor::PathPostProcessor()
{
  unit_scale_ = 1;
  ref_pt_ = Eigen::Vector3d(0, 0, 0);
  transf_vec_ = Eigen::Vector3d(0, 0, 0);
  verbose_ = false;
}

void framefab_path_post_processing::PathPostProcessor::setParams(
    framefab_msgs::ModelInputParameters model_params,
    framefab_msgs::PathInputParameters path_params)
{
  model_input_params_ = model_params;
  path_input_params_ = path_params;

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

  // set ref point
  ref_pt_ = Eigen::Vector3d(model_input_params_.ref_pt_x, model_input_params_.ref_pt_y, model_input_params_.ref_pt_z);
}

bool framefab_path_post_processing::PathPostProcessor::createCandidatePoses()
{
  using namespace rapidjson;

  /* --- 1. Parse the input JSON file into a document --- */
  std::string fpath = path_input_params_.file_path;

  FILE* fp = fopen(fpath.c_str(), "r");

  char readBuffer[65536];
  FileReadStream is(fp, readBuffer, sizeof(readBuffer));

  Document document;

  if(document.ParseStream(is).HasParseError())
  {
    ROS_ERROR_STREAM("PathPostProcessor has ERROR parsing the input json file!");
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

    // create UnitProcessPath & Add UnitProcessPath into ProcessPath
    path_array_.push_back(createScaledUnitProcessPath(i, st_pt, end_pt, feasible_orients, type_str, 0.005));
  }

  ROS_INFO_STREAM("path json Parsing succeeded.");
  return true;
}

framefab_utils::UnitProcessPath framefab_path_post_processing::PathPostProcessor::createScaledUnitProcessPath(
    int index, Eigen::Vector3d st_pt, Eigen::Vector3d end_pt,
    std::vector<Eigen::Vector3d> feasible_orients,
    std::string type_str,
    double shrink_length)
{
  framefab_utils::UnitProcessPath upp(
      index,
      transformPoint(st_pt, unit_scale_, transf_vec_),
      transformPoint(end_pt, unit_scale_, transf_vec_),
      feasible_orients, type_str, shrink_length);

  return upp;
}
