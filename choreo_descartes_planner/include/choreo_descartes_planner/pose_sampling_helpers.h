//
// Created by yijiangh on 4/9/18.
//

#ifndef FRAMEFAB_MPP_POSE_SAMPLING_HELPERS_H
#define FRAMEFAB_MPP_POSE_SAMPLING_HELPERS_H

#include "choreo_descartes_planner/capsulated_ladder_tree.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace descartes_planner
{

std::vector<Eigen::Vector3d> discretizePositions(
    const Eigen::Vector3d& start, const Eigen::Vector3d& stop, const double ds);

Eigen::Affine3d makePose(const Eigen::Vector3d& position, const Eigen::Matrix3d& orientation,
                         const double z_axis_angle);

Eigen::Affine3d makePose(const Eigen::Vector3d& position, const Eigen::Matrix3d& orientation);

int randomSampleInt(int lower, int upper);

double randomSampleDouble(double lower, double upper);

std::vector<Eigen::Affine3d> generateSample(const descartes_planner::CapRung& cap_rung,
                                            descartes_planner::CapVert& cap_vert);


std::vector<std::vector<Eigen::Affine3d>> generateSamplePickNPlace(const descartes_planner::CapRung& cap_rung,
                                                                   descartes_planner::CapVert& cap_vert);

}

#endif //FRAMEFAB_MPP_POSE_SAMPLING_HELPERS_H
