//
// Created by yijiangh on 4/7/18.
//

#ifndef FRAMEFAB_MPP_JSON2MSG_HELPER_H
#define FRAMEFAB_MPP_JSON2MSG_HELPER_H

#include <framefab_rapidjson/include/rapidjson/document.h>

// msgs
#include <framefab_msgs/Grasp.h>

namespace framefab_task_sequence_processing
{

bool isValidJsonGHVector(const rapidjson::Value& j_vector);

bool isValidJsonGHPlane(const rapidjson::Value& j_plane);

void jsonToGraspFrameFabMsg(const rapidjson::Value& json, framefab_msgs::Grasp& g);

}

#endif //FRAMEFAB_MPP_JSON2MSG_HELPER_H
