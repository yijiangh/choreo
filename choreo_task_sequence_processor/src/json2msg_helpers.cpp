//
// Created by yijiangh on 4/7/18.
//

#include "choreo_task_sequence_processor/json2msg_helpers.h"

#include <choreo_geometry_conversion_helpers/choreo_geometry_conversion_helpers.h>

namespace {
// the validity of input json data should be checked in the top level function
void jsonVectorToEigenVectorImpl(const rapidjson::Value& json, Eigen::Vector3d& v)
{
  v = Eigen::Vector3d(json["X"].GetDouble(), json["Y"].GetDouble(), json["Z"].GetDouble());
}

void jsonPlaneToPoseMsgImpl(const rapidjson::Value& json, geometry_msgs::Pose& p)
{
  Eigen::Vector3d o, x_axis, y_axis, z_axis;

  if(json.HasMember("OriginX") && json.HasMember("OriginY") && json.HasMember("OriginZ"))
  {
    o = Eigen::Vector3d(json["OriginX"].GetDouble(), json["OriginY"].GetDouble(), json["OriginZ"].GetDouble());
  }
  else
  {
    // use "Origin" attribute
    jsonVectorToEigenVectorImpl(json["Origin"], o);
  }

  jsonVectorToEigenVectorImpl(json["XAxis"], x_axis);
  jsonVectorToEigenVectorImpl(json["YAxis"], y_axis);
  jsonVectorToEigenVectorImpl(json["ZAxis"], z_axis);

  choreo_geometry_conversion_helpers::planeToPoseMsg(o, x_axis, y_axis, z_axis, p);
}

} // anon util namespace

namespace choreo_task_sequence_processing
{

bool isValidJsonGHVector(const rapidjson::Value& j_vector)
{
  if(j_vector.HasMember("X") && j_vector.HasMember("Y") && j_vector.HasMember("Z"))
  {
    if(j_vector["X"].IsDouble() && j_vector["Y"].IsDouble() && j_vector["Z"].IsDouble())
    {
      return true;
    }
  }

  return false;
}

bool isValidJsonGHPlane(const rapidjson::Value& j_plane)
{
  bool has_valid_origin = false;
  bool has_valid_axes = false;

  if(j_plane.HasMember("OriginX") && j_plane.HasMember("OriginY") && j_plane.HasMember("OriginZ"))
  {
    if(j_plane["OriginX"].IsDouble() && j_plane["OriginY"].IsDouble() && j_plane["OriginZ"].IsDouble())
    {
      has_valid_origin = true;
    }
  }
  else
  {
    if(j_plane.HasMember("Origin"))
    {
      if(isValidJsonGHVector(j_plane["Origin"]))
      {
        has_valid_origin = true;
      }
    }
  }

  if(j_plane.HasMember("XAxis") && j_plane.HasMember("YAxis") && j_plane.HasMember("ZAxis"))
  {
    has_valid_axes = isValidJsonGHVector(j_plane["XAxis"])
        && isValidJsonGHVector(j_plane["YAxis"])
        && isValidJsonGHVector(j_plane["ZAxis"]);
  }

  return has_valid_origin && has_valid_axes;
}

void jsonVectorToEigenVector(const rapidjson::Value& json, Eigen::Vector3d& v)
{
  assert(isValidJsonGHVector(json));
  jsonVectorToEigenVectorImpl(json, v);
}

void jsonPlaneToPoseMsg(const rapidjson::Value& json, geometry_msgs::Pose& p)
{
  assert(isValidJsonGHPlane(json));
  jsonPlaneToPoseMsgImpl(json, p);
}

void jsonToGraspFrameFabMsg(const rapidjson::Value& json, choreo_msgs::Grasp& g)
{
  assert(json.HasMember("pick_grasp_plane"));
  assert(json.HasMember("pick_grasp_approach_plane"));
  assert(json.HasMember("pick_grasp_retreat_plane"));

  assert(json.HasMember("place_grasp_plane"));
  assert(json.HasMember("place_grasp_approach_plane"));
  assert(json.HasMember("place_grasp_retreat_plane"));

  assert(isValidJsonGHPlane(json["pick_grasp_plane"]));
  assert(isValidJsonGHPlane(json["pick_grasp_approach_plane"]));
  assert(isValidJsonGHPlane(json["pick_grasp_retreat_plane"]));

  assert(isValidJsonGHPlane(json["place_grasp_plane"]));
  assert(isValidJsonGHPlane(json["place_grasp_approach_plane"]));
  assert(isValidJsonGHPlane(json["place_grasp_retreat_plane"]));

  jsonPlaneToPoseMsgImpl(json["pick_grasp_plane"], g.pick_grasp_pose);
  jsonPlaneToPoseMsgImpl(json["pick_grasp_approach_plane"], g.pick_grasp_approach_pose);
  jsonPlaneToPoseMsgImpl(json["pick_grasp_retreat_plane"], g.pick_grasp_retreat_pose);

  jsonPlaneToPoseMsgImpl(json["place_grasp_plane"], g.place_grasp_pose);
  jsonPlaneToPoseMsgImpl(json["place_grasp_approach_plane"], g.place_grasp_approach_pose);
  jsonPlaneToPoseMsgImpl(json["place_grasp_retreat_plane"], g.place_grasp_retreat_pose);
}

}