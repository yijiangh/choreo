//
// Created by yijiangh on 6/25/17.
//
#include <ros/ros.h>
#include <ros/console.h>

#include <framefab_path_post_processor/path_post_processor.h>

#include <rapidjson/document.h>
#include "rapidjson/filereadstream.h"

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
  ROS_INFO_STREAM("path json Parsing succeeded.");

  int m = document["element_number"].GetInt();
  ROS_INFO_STREAM("model element member: " << m);

  const Value& process_path_array = document["sequenced_elements"];
  assert(process_path_array.IsArray());

  for (SizeType i = 0; i < process_path_array.Size(); i++)
  {
    const Value& element_path = process_path_array[i];
    double st_x = element_path["start_pt"][0].GetDouble();
    double st_y = element_path["start_pt"][1].GetDouble();
    double st_z = element_path["start_pt"][2].GetDouble();

    double end_x = element_path["end_pt"][0].GetDouble();
    double end_y = element_path["end_pt"][1].GetDouble();
    double end_z = element_path["end_pt"][2].GetDouble();


//    ROS_INFO_STREAM("Node " << i << " - start pt (" << x << ", " << y << ", " << z << ")");
  }

  return true;
}
