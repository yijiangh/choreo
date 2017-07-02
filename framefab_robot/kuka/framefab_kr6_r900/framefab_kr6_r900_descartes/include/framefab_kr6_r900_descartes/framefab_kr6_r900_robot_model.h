#ifndef FRAMEFAB_KR6_R900_ROBOT_MODEL_H
#define FRAMEFAB_KR6_R900_ROBOT_MODEL_H

#include <descartes_moveit/moveit_state_adapter.h>
#include <framefab_kr6_r900_ikfast_manipulator_plugin/kuka_kr6r900sixx_manipulator_ikfast_moveit_plugin.hpp>

namespace framefab_kr6_r900_descartes
{
class FrameFabKr6R900RobotModel : public descartes_moveit::MoveitStateAdapter,
                             public framefab_kr6_r900_ikfast_manipulator_plugin::IKFastKinematicsPlugin
{
 public:
  FrameFabKr6R900RobotModel();

  virtual bool initialize(const std::string& robot_description, const std::string& group_name,
                          const std::string& world_frame, const std::string& tcp_frame);

  virtual bool getAllIK(const Eigen::Affine3d& pose,
                        std::vector<std::vector<double> >& joint_poses) const;

  virtual descartes_core::RobotModelPtr clone() const
  {
    descartes_core::RobotModelPtr ptr(new FrameFabKr6R900RobotModel());
    ptr->initialize("robot_description", descartes_moveit::MoveitStateAdapter::group_name_,
                    world_frame_, tool_frame_);
    return ptr;
  }

 protected:
  descartes_core::Frame world_to_base_; // world to arm base
  descartes_core::Frame tool_to_tip_;   // from urdf tool to arm tool
};
}

#endif