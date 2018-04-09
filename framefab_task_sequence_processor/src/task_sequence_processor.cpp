//
// Created by yijiangh on 6/25/17.
//
#include "framefab_task_sequence_processor/unit_process.h"
#include "framefab_task_sequence_processor/task_sequence_processor.h"
#include "framefab_task_sequence_processor/json2msg_helpers.h"

#include <choreo_geometry_conversion_helpers/choreo_geometry_conversion_helpers.h>

#include <framefab_rapidjson/include/rapidjson/document.h>
#include <framefab_rapidjson/include/rapidjson/filereadstream.h>

#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

#include <ros/ros.h>
#include <ros/console.h>
#include <boost/filesystem.hpp>

namespace
{
Eigen::Vector3d transformPoint(const Eigen::Vector3d &pt, const double &scale, const Eigen::Vector3d &ref_transf)
{
  return (pt * scale + ref_transf);
}

void scalePoseMsg(const double& scale, geometry_msgs::Pose& p)
{
  p.position.x *= scale;
  p.position.y *= scale;
  p.position.z *= scale;
}

void scalePoseFramefabMsg(const double& scale, framefab_msgs::Grasp& g)
{
  scalePoseMsg(scale, g.pick_grasp_pose);
  scalePoseMsg(scale, g.pick_grasp_approach_pose);
  scalePoseMsg(scale, g.pick_grasp_retreat_pose);

  scalePoseMsg(scale, g.place_grasp_pose);
  scalePoseMsg(scale, g.place_grasp_approach_pose);
  scalePoseMsg(scale, g.place_grasp_retreat_pose);
}
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
    framefab_msgs::TaskSequenceInputParameters task_sequence_params,
    std::string world_frame)
{
  model_input_params_ = model_params;
  path_input_params_ = task_sequence_params;
  world_frame_ = world_frame;

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

bool framefab_task_sequence_processing::TaskSequenceProcessor::parseAssemblySequencePickNPlace(
    const framefab_msgs::ModelInputParameters& model_params,
    const framefab_msgs::TaskSequenceInputParameters& task_sequence_params,
    const std::string& world_frame,
    framefab_msgs::AssemblySequencePickNPlace& as_pnp)
{
  this->setParams(model_params, task_sequence_params, world_frame);

  using namespace rapidjson;
  using namespace choreo_geometry_conversion_helpers;

  // https://stackoverflow.com/questions/8520560/get-a-file-name-from-a-path
  const std::string json_whole_path = task_sequence_params.file_path;
  boost::filesystem::path boost_json_path(json_whole_path);

  FILE* fp = fopen(json_whole_path.c_str(), "r");

  assert(fp);

  char readBuffer[65536];
  FileReadStream is(fp, readBuffer, sizeof(readBuffer));

  Document document;

  if(document.ParseStream(is).HasParseError())
  {
    ROS_ERROR_STREAM("TaskSequenceProcessor has ERROR parsing the input json file!");
    return false;
  }

  fclose(fp);

  assert(document.HasMember("assembly_type"));
  std::string at = document["assembly_type"].GetString();

  // wire in assembly type
  as_pnp.assembly_type = at;

  assert(document.HasMember("element_number"));
  int e_num = document["element_number"].GetInt();

  // wire in element number
  as_pnp.element_number = e_num;

  assert(document.HasMember("pick_base_center_point"));
  assert(document.HasMember("place_base_center_point"));
  const Value& pick_bcp = document["pick_base_center_point"];
  const Value& place_bcp = document["place_base_center_point"];
  Eigen::Vector3d pick_base_center_pt(pick_bcp["X"].GetDouble(), pick_bcp["Y"].GetDouble(), pick_bcp["Z"].GetDouble());
  pick_base_center_pt *= unit_scale_;
  Eigen::Vector3d place_base_center_pt(place_bcp["X"].GetDouble(), place_bcp["Y"].GetDouble(), place_bcp["Z"].GetDouble());
  place_base_center_pt *= unit_scale_;

  // wire in base center point msg TODO: this should be a frame!!
  tf::pointEigenToMsg(pick_base_center_pt, as_pnp.pick_base_center_point);
  tf::pointEigenToMsg(place_base_center_pt, as_pnp.place_base_center_point);

  // wire in file path
  as_pnp.file_path = boost_json_path.parent_path().string() + boost::filesystem::path::preferred_separator;

  // wire in support surfaces file name
  assert(document.HasMember("pick_support_surface_file_names"));
  assert(document["pick_support_surface_file_names"].IsArray());
  as_pnp.pick_support_surface_file_names.clear();
  for(int i=0; i<document["pick_support_surface_file_names"].Size();i++)
  {
    as_pnp.pick_support_surface_file_names.push_back(document["pick_support_surface_file_names"][i].GetString());
  }

  assert(document.HasMember("place_support_surface_file_names"));
  assert(document["place_support_surface_file_names"].IsArray());
  as_pnp.place_support_surface_file_names.clear();
  for(int i=0; i<document["place_support_surface_file_names"].Size();i++)
  {
    as_pnp.place_support_surface_file_names.push_back(document["place_support_surface_file_names"][i].GetString());
  }

  if(0 == document["place_support_surface_file_names"].Size()
      || 0 == document["pick_support_surface_file_names"].Size())
  {
    ROS_WARN_STREAM("task sequence processing: no pick and place support surfaces are found!");
  }

  assert(document.HasMember("sequenced_elements"));
  const Value& se_array = document["sequenced_elements"];
  assert(se_array.Size() > 0);

  as_pnp.sequenced_elements.clear();

  for (SizeType i = 0; i < se_array.Size(); i++)
  {
    framefab_msgs::SequencedElement se_msg;

    const Value& se = se_array[i];

    assert(se.HasMember("order_id"));
    se_msg.order_id = se["order_id"].GetInt();

    // https://www.boost.org/doc/libs/1_61_0/libs/filesystem/doc/reference.html
    // last character includes separator
    se_msg.file_path = boost_json_path.parent_path().string() + boost::filesystem::path::preferred_separator;

    assert(se.HasMember("pick_element_geometry_file_name"));
    se_msg.pick_element_geometry_file_name = se["pick_element_geometry_file_name"].GetString();
    // check file existence
    assert(boost::filesystem::exists(se_msg.file_path + se_msg.pick_element_geometry_file_name));

    assert(se.HasMember("place_element_geometry_file_name"));
    se_msg.place_element_geometry_file_name = se["place_element_geometry_file_name"].GetString();
    assert(boost::filesystem::exists(se_msg.file_path + se_msg.place_element_geometry_file_name));

    if(se.HasMember("pick_support_surface_file_names"))
    {
      const Value& pick_surf_names = se["pick_support_surface_file_names"];
      assert(pick_surf_names.IsArray());
      se_msg.pick_support_surface_file_names.clear();

      for (SizeType j=0; j<pick_surf_names.Size(); j++)
      {
        assert(pick_surf_names[j].IsString());
        assert(boost::filesystem::exists(se_msg.file_path + std::string(pick_surf_names[j].GetString())));
        se_msg.pick_support_surface_file_names.push_back(std::string(pick_surf_names[j].GetString()));
      }
    }

    if(se.HasMember("place_support_surface_file_names"))
    {
      const Value &place_surf_names = se["place_support_surface_file_names"];
      assert(place_surf_names.IsArray());
      se_msg.place_support_surface_file_names.clear();

      for (SizeType j=0; j<place_surf_names.Size(); j++)
      {
        assert(place_surf_names[j].IsString());
        assert(boost::filesystem::exists(se_msg.file_path + std::string(place_surf_names[j].GetString())));
        se_msg.place_support_surface_file_names.push_back(std::string(place_surf_names[j].GetString()));
      }
    }

    if(se.HasMember("pick_contact_ngh_ids"))
    {
      const Value &pick_ngh_ids = se["pick_contact_ngh_ids"];
      assert(pick_ngh_ids.IsArray());
      se_msg.pick_contact_ngh_ids.clear();

      for (SizeType j=0; j<pick_ngh_ids.Size(); j++)
      {
        assert(pick_ngh_ids[j].IsInt());
        se_msg.pick_contact_ngh_ids.push_back(pick_ngh_ids[j].GetInt());
      }
    }

    if(se.HasMember("place_contact_ngh_ids"))
    {
      const Value &place_ngh_ids = se["place_contact_ngh_ids"];
      assert(place_ngh_ids.IsArray());
      se_msg.place_contact_ngh_ids.clear();

      for (SizeType j=0; j<place_ngh_ids.Size(); j++)
      {
        assert(place_ngh_ids[j].IsInt());
        se_msg.place_contact_ngh_ids.push_back(place_ngh_ids[j].GetInt());
      }
    }

    // data must include at least one grasp
    assert(se.HasMember("grasps"));
    assert(se["grasps"].IsArray() && se["grasps"].Size() > 0);

    se_msg.grasps.clear();

    for(SizeType j=0; j<se["grasps"].Size(); j++)
    {
      framefab_msgs::Grasp g_msg;

      jsonToGraspFrameFabMsg(se["grasps"][j], g_msg);
      scalePoseFramefabMsg(unit_scale_, g_msg);

      se_msg.grasps.push_back(g_msg);
    }

    as_pnp.sequenced_elements.push_back(se_msg);
  }

//  ROS_INFO_STREAM(as_pnp);

  ROS_INFO_STREAM("[task sequence processor] assembly sequence [PICKNPLCAE] json parsing succeeded.");
  return true;
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

  assert(document.HasMember("element_number"));
  int m = document["element_number"].GetInt();

  assert(document.HasMember("base_center_pt"));
  const Value& bcp = document["base_center_pt"];
  Eigen::Vector3d base_center_pt(bcp[0].GetDouble(), bcp[1].GetDouble(), bcp[2].GetDouble());

  if(verbose_)
  {
    ROS_INFO_STREAM("model element member: " << m);
    ROS_INFO_STREAM("base_center_pt: \n" << base_center_pt);
  }

  setTransfVec(ref_pt_, base_center_pt, unit_scale_);

  assert(document.HasMember("sequenced_elements"));
  const Value& process_path_array = document["sequenced_elements"];
  assert(process_path_array.IsArray());

  path_array_.clear();

  for (SizeType i = 0; i < process_path_array.Size(); i++)
  {
    const Value& element_path = process_path_array[i];

    assert(element_path.HasMember("start_pt"));
    Eigen::Vector3d st_pt(element_path["start_pt"][0].GetDouble(),
                          element_path["start_pt"][1].GetDouble(),
                          element_path["start_pt"][2].GetDouble());

    assert(element_path.HasMember("end_pt"));
    Eigen::Vector3d end_pt(element_path["end_pt"][0].GetDouble(),
                           element_path["end_pt"][1].GetDouble(),
                           element_path["end_pt"][2].GetDouble());

    assert(element_path.HasMember("type"));
    std::string type_str = element_path["type"].GetString();

    assert(element_path.HasMember("wireframe_id"));
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

    assert(element_path.HasMember("feasible_orientation"));
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

  ROS_INFO_STREAM("[task sequence processor] task sequence json [spatial extrusion] parsing succeeded.");
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
  collision_env_obj.header.frame_id = world_frame_;
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

framefab_task_sequence_processing_utils::UnitProcess
framefab_task_sequence_processing::TaskSequenceProcessor::createScaledUnitProcess(
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
      feasible_orients, type_str, element_diameter, shrink_length, world_frame_);

  return upp;
}
