//
// Created by yijiangh on 7/10/17.
//

#ifndef FRAMEFAB_PROCESS_EXECUTION_PROCESS_UTILS_H
#define FRAMEFAB_PROCESS_EXECUTION_PROCESS_UTILS_H

#include <trajectory_msgs/JointTrajectory.h>

namespace framefab_process_execution
{

void appendTrajectory(trajectory_msgs::JointTrajectory& original,
                      const trajectory_msgs::JointTrajectory& next);
}

#endif //FRAMEFAB_PROCESS_EXECUTION_PROCESS_UTILS_H
