//
// Created by yijiangh on 7/10/17.
//

#ifndef FRAMEFAB_EXECUTION_GATEKEEPER_UTILS_H
#define FRAMEFAB_EXECUTION_GATEKEEPER_UTILS_H

#include <trajectory_msgs/JointTrajectory.h>

namespace framefab_execution_gatekeeper
{

void appendTrajectory(trajectory_msgs::JointTrajectory& original,
                      const trajectory_msgs::JointTrajectory& next);
}

#endif //FRAMEFAB_EXECUTION_GATEKEEPER_UTILS_H
