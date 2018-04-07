//
// Created by yijiangh on 4/7/18.
//

#include "framefab_task_sequence_processor/json2msg_helpers.h"

namespace {


} // anon util namespace

namespace framefab_task_sequence_processing
{

bool isValidJsonGHVector(const rapidjson::Value& j_vector)
{
  if(j_vector.HasMember("X") && j_vector.HasMember("Y") && j_vector.HasMember("Z"))
  {
    if(j_vector["Z"].IsDouble() && j_vector["Y"].IsDouble() && j_vector["Z"].IsDouble())
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

  if(j_plane.HasMember("OriginX") && j_plane.HasMember("OriginY") && j_plane.HasMember("OriginX"))
  {
    if(j_plane["OriginX"].IsDouble() && j_plane["OriginX"].IsDouble() && j_plane["OriginX"].IsDouble())
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
        && isValidJsonGHVector(j_plane["YAxis"]);
  }

  return has_valid_origin && has_valid_axes;
}

void jsonToGraspFrameFabMsg(const rapidjson::Value& json, framefab_msgs::Grasp& g)
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
}
}