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

  /* --- 2. Access values in the document, create processPath out of them --- */
//  static const char* kTypeNames[] = { "Null", "False", "True", "Object", "Array", "String", "Number" };
  for (Value::ConstMemberIterator itr = document.MemberBegin(); itr != document.MemberEnd(); ++itr)
  {
    ROS_INFO_STREAM("Json member %s" << itr->name.GetString());
  }
//    printf("Type of member %s is %s\n", itr->name.GetString(), kTypeNames[itr->value.GetType()]);

  return true;
}
