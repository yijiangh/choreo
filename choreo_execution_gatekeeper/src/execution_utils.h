//
// Created by yijiangh on 7/10/17.
//

#ifndef FRAMEFAB_EXECUTION_GATEKEEPER_UTILS_H
#define FRAMEFAB_EXECUTION_GATEKEEPER_UTILS_H

#include <trajectory_msgs/JointTrajectory.h>

namespace choreo_execution_gatekeeper
{
// helper function to rebase and concatenate the timestamps of the input joint trajectory
//
// input: next: trajectory chunks to be appended, starting time stamp is not synchronized
// output: original: trajectories to be appended upon
void appendTrajectory(trajectory_msgs::JointTrajectory& original,
                      const trajectory_msgs::JointTrajectory& next);
}

#endif //FRAMEFAB_EXECUTION_GATEKEEPER_UTILS_H
