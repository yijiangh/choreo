//
// Created by yijiangh on 7/5/17.
//

#include <framefab_process_planning/framefab_process_planning.h>
#include <ros/console.h>

#include <framefab_msgs/ElementCandidatePoses.h>

// descartes
#include "descartes_trajectory/cart_trajectory_pt.h"
#include "descartes_planner/dense_planner.h"

//#include "path_transitions.h"
//#include "common_utils.h"
//#include "generate_motion_plan.h"

namespace framefab_process_planning
{
// Planning Constants
const double PRINT_ANGLE_DISCRETIZATION =
    M_PI / 12.0; // The discretization of the tool's pose about the z axis

const static std::string JOINT_TOPIC_NAME =
    "joint_states"; // ROS topic to subscribe to for current robot state info

descartes_core::TrajectoryPtPtr toDescartesPrintPt(const Eigen::Affine3d& pose, double dt)
{
  using namespace descartes_trajectory;
  using namespace descartes_core;
  const TimingConstraint tm(dt);
  return boost::make_shared<AxialSymmetricPt>(pose, PRINT_ANGLE_DISCRETIZATION,
                                              AxialSymmetricPt::Z_AXIS, tm);
}

bool ProcessPlanningManager::handlePrintPlanning(godel_msgs::ProcessPlanning::Request& req,
                                                 godel_msgs::ProcessPlanning::Response& res)
{
  // Enable Collision Checks
  blend_model_->setCheckCollisions(true);

  std::vector<framefab_msgs::ElementCandidatePoses> process_path = req.process_path;

  if (process_path.empty())
  {
    ROS_WARN("Planning request contained no process path. Nothing to be done.");
    return true;
  }

  // Transform process path from geometry msgs to descartes points
  std::vector<double> current_joints = getCurrentJointState(JOINT_TOPIC_NAME);

  const static double LINEAR_DISCRETIZATION = 0.01; // meters
  const static double ANGULAR_DISCRETIZATION = 0.1; // radians
  const static double RETRACT_DISTANCE = 0.05; // meters

  TransitionParameters transition_params;
  transition_params.linear_disc = LINEAR_DISCRETIZATION;
  transition_params.angular_disc = ANGULAR_DISCRETIZATION;
  transition_params.retract_dist = RETRACT_DISTANCE;

//  DescartesTraj process_points = toDescartesTraj(req.path.segments, req.params.traverse_spd, transition_params,
//                                                 toDescartesBlendPt);

//  generateMotionPlan(blend_model_, process_points, moveit_model_, blend_group_name_,
//                         current_joints, res.plan)

  return true;
}
}