#ifndef CHOREO_DESCARTES_GRAPH_BUILDER_H
#define CHOREO_DESCARTES_GRAPH_BUILDER_H

#include <Eigen/Geometry>
#include <eigen_stl_containers/eigen_stl_vector_container.h>
#include <descartes_planner/ladder_graph.h>
#include <descartes_core/robot_model.h>

/* So we want to represent each extrusion as a single process graph that could have many
 * start and end points but must stay inside one configuration through the process.
 *
 * For the moment, we do not consider the motion planning problem
 */
namespace descartes_planner
{

struct ConstrainedSegment
{
  enum PROCESS_TYPE
  {
    CREATE,
    CONNECT,
    SUPPORT
  };

  using OrientationVector = std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d>>;

  Eigen::Vector3d start, end; /** Start and end of the linear segment in 3-space */
  OrientationVector orientations; /** All of the allowable orientations of the tool for this path */
  double linear_disc; /** The distance between sampled points in linear space (I'd like to do this automatically) */
  double linear_vel; /** Linear velocity between each disc point */
  double z_axis_disc; /** The distance between angular steps about z for each orientation */
  double retract_dist;

  PROCESS_TYPE process_type; /** create type, one node is floating; connect type, both of the nodes have been printed */

  int retract_start_pt_num;
  int process_pt_num;
  int retract_end_pt_num;
};

LadderGraph sampleSingleConfig(const descartes_core::RobotModel& model,
                               const std::vector<Eigen::Vector3d>& ps,
                               const double dt, const Eigen::Matrix3d& orientation,
                               const double z_axis_angle);

LadderGraph sampleConstrainedPaths(const descartes_core::RobotModel& model, ConstrainedSegment& segment);

// TODO: should be renamed
// Appends 'next' to the end of 'current' to produce a new graph
void appendInTime(LadderGraph& current, const LadderGraph& next);

}

#endif // GRAPH_BUILDER_H
