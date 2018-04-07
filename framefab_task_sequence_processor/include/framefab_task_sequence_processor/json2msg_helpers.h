//
// Created by yijiangh on 4/7/18.
//

#ifndef FRAMEFAB_MPP_JSON2MSG_HELPER_H
#define FRAMEFAB_MPP_JSON2MSG_HELPER_H

#include <framefab_rapidjson/include/rapidjson/document.h>

#include <Eigen/Core>

// msgs
#include <framefab_msgs/Grasp.h>
#include <geometry_msgs/Pose.h>

namespace framefab_task_sequence_processing
{

bool isValidJsonGHVector(const rapidjson::Value& j_vector);

bool isValidJsonGHPlane(const rapidjson::Value& j_plane);

void jsonVectorToEigenVector(const rapidjson::Value& json, Eigen::Vector3d& v);

void jsonPlaneToPoseMsg(const rapidjson::Value& json, geometry_msgs::Pose& p);

void jsonToGraspFrameFabMsg(const rapidjson::Value& json, framefab_msgs::Grasp& g);

}

#endif //FRAMEFAB_MPP_JSON2MSG_HELPER_H
