//
// Created by yijiangh on 7/9/17.
//

#ifndef CHOREO_PROCESS_PLANNING_TRAJECTORY_UTILS_H
#define CHOREO_PROCESS_PLANNING_TRAJECTORY_UTILS_H

#include <Eigen/Geometry>

namespace choreo_process_planning
{
// Cartesian Interpolation
typedef std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d> > PoseVector;

PoseVector interpolateCartesian(const Eigen::Affine3d& start, const Eigen::Affine3d& stop,
                                double ds);
// Joint Interpolation
typedef std::vector<std::vector<double> > JointVector;

JointVector interpolateJoint(const std::vector<double>& start, const std::vector<double>& stop,
                             double dtheta);
}

#endif //CHOREO_PROCESS_PLANNING_TRAJECTORY_UTILS_H
