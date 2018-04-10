#ifndef CHOREO_DESCARTES_GRAPH_BUILDER_H
#define CHOREO_DESCARTES_GRAPH_BUILDER_H

#include "constrained_segment.h"

#include <descartes_planner/ladder_graph.h>
#include <descartes_core/robot_model.h>

#include <Eigen/Geometry>

/*
 * JM:
 * So we want to represent each extrusion as a single process graph that could have many
 * start and end points but must stay inside one configuration through the process.
 *
 * For the moment, we do not consider the motion planning problem
 */
namespace descartes_planner
{

LadderGraph generateLadderGraphFromPoses(const descartes_core::RobotModel& model,
                                         const std::vector<Eigen::Affine3d>& ps);

LadderGraph sampleSingleConfig(const descartes_core::RobotModel& model,
                               const std::vector<Eigen::Vector3d>& origins,
                               const double dt,
                               const Eigen::Matrix3d& orientation,
                               const double z_axis_angle);

// ARCHIVED
// author: J. Meyer (SWRI) with some edits by Yijiang Huang (yijiangh@mit.edu)
// fully-sampled ladder graph from a given z-axis discretized constrained segment
// (so you can get an *optimal* solution in the given discretization if you perform graph search on it.)
// in the light of the CLT-RRT, this scheme is rarely used and is archived here.
LadderGraph sampleConstrainedPaths(const descartes_core::RobotModel& model, ConstrainedSegment& segment);

// TODO: should be renamed
// Appends 'next' to the end of 'current' to produce a new graph
void appendInTime(LadderGraph& current, const LadderGraph& next);

}

#endif // GRAPH_BUILDER_H
