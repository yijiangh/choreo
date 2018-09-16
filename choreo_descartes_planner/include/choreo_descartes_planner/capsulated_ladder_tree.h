//
// Created by yijiangh on 11/28/17.
//

#ifndef CHOREO_DESCARTES_CAPSULATED_LADDER_TREE_H
#define CHOREO_DESCARTES_CAPSULATED_LADDER_TREE_H

// for Constrained Segment
#include "choreo_ladder_graph_builder.h"

#include <moveit/planning_scene/planning_scene.h>

const static double ORIENTATION_PREFERENCE_WEIGHT = 0.0;
const static double EXTERNAL_AXIS_PENALIZE_COST = 1.0;

namespace descartes_planner
{

class CapVert
{
 public:
  explicit CapVert(const double dof) : dof_(dof), z_axis_angle_(-1)
  {
  }

  ~CapVert()
  {}

  inline double distance(CapVert* v) const
  {
    // sanity check
    if(NULL == v)
    {
      return 0.0;
    }

    assert(v->dof_ == this->dof_);

    // directed distance prev.end -> this.start
    const auto dof = this->dof_;
    double cost = std::numeric_limits<double>::max();
    const auto n_prev_end = v->end_joint_data_.size() / dof;
    const auto n_this_start = this->start_joint_data_.size() / dof;

    for(size_t i=0; i < n_prev_end; i++)
    {
      const auto prev_end_index = i * dof;

      for (size_t j=0; j < n_this_start; j++)
      {
        const auto this_start_index = j * dof;

        std::vector<double> delta_buffer;

        if(dof <= 6)
        {
          for (size_t k = 0; k < dof; k++)
          {
            delta_buffer.push_back(std::abs(v->end_joint_data_[prev_end_index + k]
                                                - this->start_joint_data_[this_start_index + k]));
          }
        }
        else
        {
          // penalize linear track movement
          for (size_t k = 0; k < dof; k++)
          {
            double axis_weight = 1.0;

            if(k < dof - 6)
            {
              axis_weight = EXTERNAL_AXIS_PENALIZE_COST;
            }

            delta_buffer.push_back(axis_weight *
                std::abs(v->end_joint_data_[prev_end_index + k]
                             - this->start_joint_data_[this_start_index + k]));
          }
        }

        double tmp_cost = std::accumulate(delta_buffer.begin(), delta_buffer.end(), 0.0);

        if(tmp_cost < cost)
        {
          cost = tmp_cost;
        }
      }
    }

    return cost;
  }

  inline double getToParentCost() { return this->to_parent_cost_; }

  inline CapVert* getParentVertPtr() const { return this->ptr_prev_cap_vert_; }

  inline void setParentVertPtr(CapVert* ptr_v)
  {
    this->ptr_prev_cap_vert_ = ptr_v;
    this->to_parent_cost_ = this->distance(ptr_v);
//    + ORIENTATION_PREFERENCE_WEIGHT * delta_o_to_ideal_angle_;
  }

  // accumulated cost (the function g)
  inline double getCost() const
  {
    // trace back tree to compute cost
    CapVert* ptr_prev_v = this->ptr_prev_cap_vert_;
    double cost = this->to_parent_cost_;

    while(NULL != ptr_prev_v)
    {
      if(NULL != ptr_prev_v->ptr_prev_cap_vert_)
      {
        cost += ptr_prev_v->getToParentCost();
      }
      ptr_prev_v = ptr_prev_v->ptr_prev_cap_vert_;
    }

    return cost;
  }

 public:
  int dof_;

  // joint values stored in one contiguous array
  // first joint pose in first partition
  std::vector<double> start_joint_data_;
  // last joint pose in last partition
  std::vector<double> end_joint_data_;

  size_t rung_id_;

  // rotation angle around z axis, TODO: only used in spatial extrusion
  // TODO: temporarily used to distinguish two processes
  double z_axis_angle_;

  // base orientation, the chosen orientation for each kinematics segment
  std::vector<Eigen::Matrix3d> orientation_;

//  double delta_o_to_ideal_angle_;

  double to_parent_cost_;
  CapVert* ptr_prev_cap_vert_;
};

struct CapRung
{
  std::vector<CapVert*> ptr_cap_verts_;

  std::vector<std::vector<Eigen::Vector3d>> path_pts_;

  // stores all the candidate orientations for each kinematics segment (family)
  std::vector<std::vector<Eigen::Matrix3d>> orientations_;

  // partition of path points inside (needs to be divided later)
  std::vector<int> sub_segment_ids_;

  // associated planning for each partition
  std::vector<planning_scene::PlanningScenePtr> planning_scene_;

  // TODO: this is temporal patch to add element that is being printed
  std::vector<planning_scene::PlanningScenePtr> planning_scene_completed_;

  // ONLY USED in spatial extrusion
  // discretization degree for rotation around central z axis
  double z_axis_disc_;

  // used in line movement discretization
  double linear_vel_;
};

}

#endif //DESCARTES_CAPSULATED_LADDER_TREE_H
