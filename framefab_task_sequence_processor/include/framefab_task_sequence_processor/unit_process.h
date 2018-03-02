//
// Created by yijiangh on 6/26/17.
//

#ifndef FRAMEFAB_PATH_POST_PROCESSOR_PROCESSPATH_H
#define FRAMEFAB_PATH_POST_PROCESSOR_PROCESSPATH_H

#include <framefab_msgs/ElementCandidatePoses.h>
#include <moveit_msgs/CollisionObject.h>
#include <Eigen/Core>

namespace framefab_task_sequence_processing_utils
{

class UnitProcess
{
 public:
  UnitProcess(int index,
              int wireframe_id,
              Eigen::Vector3d st_pt, Eigen::Vector3d end_pt,
              std::vector<Eigen::Vector3d> feasible_orients,
              std::string type_str,
              double element_diameter,
              double shrink_length)
  {
    id_ = index;
    wireframe_id_ = wireframe_id;
    st_pt_ = st_pt;
    end_pt_ = end_pt;
    feasible_orients_ = feasible_orients;
    type_ = type_str;
    element_diameter_ = element_diameter;
    shrink_length_ = shrink_length;
  }
  virtual ~UnitProcess(){}

  Eigen::Vector3d getStartPt() { return st_pt_; }
  Eigen::Vector3d getEndPt() { return end_pt_; }
  std::vector<Eigen::Vector3d> getFeasibleOrients() { return feasible_orients_; }
  std::string getType() { return type_; }

  framefab_msgs::ElementCandidatePoses asElementCandidatePoses();

 protected:
  moveit_msgs::CollisionObject createCollisionObject(
      const int& id, const Eigen::Vector3d& st_pt, const Eigen::Vector3d& end_pt,
      const double& element_diameter, std::string&& shrink_type_suffix = "") const;

  void createShrinkedEndPoint(Eigen::Vector3d& st_pt, Eigen::Vector3d& end_st,
                              double shrink_length);

  geometry_msgs::Pose computeCylinderPose(
      const Eigen::Vector3d& st_pt, const Eigen::Vector3d& end_pt) const;

 private:
  int id_;
  int wireframe_id_;
  Eigen::Vector3d st_pt_;
  Eigen::Vector3d end_pt_;
  std::vector<Eigen::Vector3d> feasible_orients_;
  std::string type_;

  // collision objects
  double element_diameter_;
  double shrink_length_;
};
}

#endif //FRAMEFAB_PATH_POST_PROCESSOR_PROCESSPATH_H
