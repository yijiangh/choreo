//
// Created by yijiangh on 6/26/17.
//

#ifndef FRAMEFAB_PATH_POST_PROCESSOR_PROCESSPATH_H
#define FRAMEFAB_PATH_POST_PROCESSOR_PROCESSPATH_H

#include <framefab_msgs/ElementCandidatePoses.h>
#include <moveit_msgs/CollisionObject.h>
#include <Eigen/Core>

namespace framefab_utils
{

class UnitProcessPath
{
 public:
  UnitProcessPath(int index,
                  Eigen::Vector3d st_pt, Eigen::Vector3d end_pt,
                  std::vector<Eigen::Vector3d> feasible_orients,
                  std::string type_str,
                  double shrink_length)
  {
    id_ = index;
    st_pt_ = st_pt;
    end_pt_ = end_pt;
    feasible_orients_ = feasible_orients;
    type_ = type_str;
    shrink_length_ = shrink_length;
  }
  virtual ~UnitProcessPath(){}

  Eigen::Vector3d getStartPt() { return st_pt_; }
  Eigen::Vector3d getEndPt() { return end_pt_; }
  std::vector<Eigen::Vector3d> getFeasibleOrients() { return feasible_orients_; }
  std::string getType() { return type_; }

  framefab_msgs::ElementCandidatePoses asElementCandidatePoses();

 protected:
  moveit_msgs::CollisionObject createCollisionObject(
      const int& id, const Eigen::Vector3d& st_pt, const Eigen::Vector3d& end_pt) const;

  void createShrinkedEndPoint(Eigen::Vector3d& st_pt, Eigen::Vector3d& end_st,
                              const double& shrink_length);

  geometry_msgs::Pose computeCylinderPose(
      const Eigen::Vector3d& st_pt, const Eigen::Vector3d& end_pt) const;

 private:
  int id_;
  Eigen::Vector3d st_pt_;
  Eigen::Vector3d end_pt_;
  std::vector<Eigen::Vector3d> feasible_orients_;
  std::string type_;

  // collision objects
  double shrink_length_; // meter
};
}

#endif //FRAMEFAB_PATH_POST_PROCESSOR_PROCESSPATH_H
