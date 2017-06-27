//
// Created by yijiangh on 6/26/17.
//

#ifndef FRAMEFAB_PATH_POST_PROCESSOR_PROCESSPATH_H
#define FRAMEFAB_PATH_POST_PROCESSOR_PROCESSPATH_H

#include <framefab_msgs/ElementCandidatePoses>
#include <moveit_msgs/CollisionObject.h>

namespace framefab_utils
{

class UnitProcessPath
{
 public:
  UnitProcessPath(){};
  virtual UnitProcessPath(){};

  void createUnitProcessPath(Eigen::Vector3d st_pt, Eigen::Vector3d end_pt,
                             std::vector<Eigen::Vector3d> feasible_orients,
                             std::string type_str,
                             double shrink_length = 0.01)
  {
    st_pt_ = st_pt;
    end_pt_ = end_pt;
    feasible_orients_ = feasible_orients;
    type_ = type_str;
    shrink_length_ = shrink_length;
  }

  Eigen::Vector3d& getStartPt() const { return st_pt_; }
  Eigen::Vector3d& getEndPt() const { return end_pt_; }
  std::vector<Eigen::Vector3d>& getFeasibleOrients() const { return feasible_orients_; }
  std::string& getType() const { return type_; }

  framefab_msgs::ElementCandidatePoses () const;

 protected:
  void createCollisionObject();
  geometry_msgs::Pose computeCylinderPose() const;

 private:
  // st pt, end pt, feasible orients, type
  Eigen::Vector3d st_pt_;
  Eigen::Vector3d end_pt_;
  std::vector<Eigen::Vector3d> feasible_orients_;
  std::string type_;

  // collision objects
  double shrink_length_; // meters
  moveit_msgs::CollisionObject collision_cylinder_;
};

class ProcessPath
{
  typedef std::vector<ElementCandidatePoses> ElementCandidatePosesArray;

 public:
  ProcessPath(){};
  virtual ~ProcessPath(){};

  void addUnitProcessPath(const UnitProcessPath& u_pth) { process_paths_.push_back(u_pth); }

  std::vector<UnitProcessPath> data() const
  {
    return process_paths_;
  }

  void clear()
  {
    process_paths_.clear();
  }

  ElementCandidatePosesArray asElementCandidatePosesArray() const;

private:
  std::vector<UnitProcessPath> process_paths_;
};
}

#endif //FRAMEFAB_PATH_POST_PROCESSOR_PROCESSPATH_H
