//
// Created by yijiangh on 6/5/17.
//
#include <framefab_process_path_generation/process_path_generator.h>

namespace framefab_process_path
{

std::vector<Eigen::Affine3d> ProcessPathGenerator::addLinePathPts(
    Eigen::Affine3d start_pose, Eigen::Affine3d end_pose, int steps)
{
  Eigen::Vector3d translation_vector;
  translation_vector = end_pose.translation() - start_pose.translation();
  translation_vector = translation_vector / steps;
  Eigen::Translation<double,3> translate(translation_vector);

  std::vector<Eigen::Affine3d> poses;
  poses.push_back(start_pose);
  for(int i = 0; i < (steps - 1); ++i)
  {
    poses.push_back(translate * poses.back());
  }
  return poses;
}

}
