//
// Created by yijiangh on 4/9/18.
//

#include "pose_sampling_helpers.h"

// opened for external usage without linking to library
namespace descartes_planner
{

std::vector<Eigen::Vector3d> discretizePositions(const Eigen::Vector3d& start, const Eigen::Vector3d& stop, const double ds)
{
  auto dist = (stop - start).norm();

  size_t n_intermediate_points = 0;
  if (dist > ds)
  {
    n_intermediate_points = static_cast<size_t>(std::lround(dist / ds));
  }

  const auto total_points = 2 + n_intermediate_points;

  std::vector<Eigen::Vector3d> result;
  result.reserve(total_points);

  for (std::size_t i = 0; i < total_points; ++i)
  {
    const double r = i / static_cast<double>(total_points - 1);
    Eigen::Vector3d point = start + (stop - start) * r;
    result.push_back(point);
  }
  return result;
}

Eigen::Affine3d makePose(const Eigen::Vector3d& position, const Eigen::Matrix3d& orientation,
                         const double z_axis_angle)
{
  Eigen::Affine3d m = Eigen::Affine3d::Identity();
  m.matrix().block<3,3>(0,0) = orientation;
  m.matrix().col(3).head<3>() = position;

  Eigen::AngleAxisd z_rot (z_axis_angle, Eigen::Vector3d::UnitZ());

  return m * z_rot;
}

Eigen::Affine3d makePose(const Eigen::Vector3d& position, const Eigen::Matrix3d& orientation)
{
  Eigen::Affine3d m = Eigen::Affine3d::Identity();
  m.matrix().block<3,3>(0,0) = orientation;
  m.matrix().col(3).head<3>() = position;

  return m;
}

int randomSampleInt(int lower, int upper)
{
  std::random_device rd;
  std::mt19937 gen(rd());

  if(upper > lower)
  {
    std::uniform_int_distribution<int> int_distr(lower, upper);
    return int_distr(gen);
  }
  else
  {
    return lower;
  }
}

double randomSampleDouble(double lower, double upper)
{
  std::random_device rd;
  std::mt19937 gen(rd());

  if(upper > lower)
  {
    std::uniform_real_distribution<double> double_distr(lower, upper);
    return double_distr(gen);
  }
  else
  {
    return lower;
  }
}

// TODO: original generate sampling function used only in spatial extrusion
std::vector<Eigen::Affine3d> generateSample(const descartes_planner::CapRung& cap_rung,
                                            descartes_planner::CapVert& cap_vert)
{
  // sample int for orientation
  int o_sample = randomSampleInt(0, cap_rung.orientations_[0].size()-1);

  assert(cap_rung.orientations_.size() > 0);
  assert(cap_rung.orientations_[0].size() > o_sample);

  Eigen::Matrix3d orientation_sample = cap_rung.orientations_[0][o_sample];

  // sample [0,1] for axis, z_axis_angle = b_rand * 2 * Pi

  double x_axis_sample = randomSampleDouble(0.0, 1.0) * 2 * M_PI;

  assert(cap_rung.path_pts_.size() > 0);
  assert(cap_rung.path_pts_[0].size() > 0);

  std::vector<Eigen::Affine3d> poses;
  poses.reserve(cap_rung.path_pts_[0].size());
  for(const auto& pt : cap_rung.path_pts_[0])
  {
    poses.push_back(makePose(pt, orientation_sample, x_axis_sample));
  }

  // TODO: should be a std::vector too?
  cap_vert.z_axis_angle_ = x_axis_sample;

  // spatial extrusion only gets one kinematics segment
  cap_vert.orientation_.resize(1);
  cap_vert.orientation_[0] = orientation_sample;

  return poses;
}

// TODO: should be more universal, not only for pnp
std::vector<std::vector<Eigen::Affine3d>> generateSamplePickNPlace(const descartes_planner::CapRung& cap_rung,
                                                                   descartes_planner::CapVert& cap_vert)
{
  // sample int for orientation
  int o_sample = randomSampleInt(0, cap_rung.orientations_[0].size()-1);

  // this index is applied to orientation candidates in each kinematics family
  assert(cap_rung.path_pts_.size() == cap_rung.sub_segment_ids_.size());

  cap_vert.orientation_.resize(cap_rung.path_pts_.size());
  std::vector<std::vector<Eigen::Affine3d>> poses(cap_rung.path_pts_.size());

  // for each kinematics family
  for(int i=0; i<cap_rung.path_pts_.size(); i++)
  {
    assert(cap_rung.orientations_.size() > 0);
    assert(cap_rung.orientations_[0].size() > o_sample);

    const auto orient = cap_rung.orientations_[i][o_sample];

    cap_vert.orientation_[i] = orient;

    poses[i].reserve(cap_rung.path_pts_[0].size());

    for(const auto &pt : cap_rung.path_pts_[i])
    {
      poses[i].push_back(makePose(pt, orient));
    }
  }

  return poses;
}

}