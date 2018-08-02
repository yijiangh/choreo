//
// Created by yijiangh on 4/9/18.
//

#ifndef CHOREO_MPP_CONSTRUCT_PLANNING_SCENE_H
#define CHOREO_MPP_CONSTRUCT_PLANNING_SCENE_H

#include <choreo_msgs/AssemblySequencePickNPlace.h>
#include <choreo_msgs/WireFrameCollisionObject.h>
#include <choreo_msgs/UnitProcessPlan.h>

#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/AttachedCollisionObject.h>

#include <moveit/planning_scene/planning_scene.h>
#include <descartes_core/robot_model.h>

namespace choreo_process_planning
{
void constructPlanningScene(const planning_scene::PlanningScenePtr base_scene,
                            const std::vector<moveit_msgs::CollisionObject>& add_cos,
                            const std::vector<moveit_msgs::CollisionObject>& remove_cos,
                            const std::vector<moveit_msgs::AttachedCollisionObject>& attach_objs,
                            const std::vector<moveit_msgs::AttachedCollisionObject>& detach_objs,
                            planning_scene::PlanningScenePtr s);

planning_scene::PlanningScenePtr constructPlanningScene(const planning_scene::PlanningScenePtr base_scene,
                                                        const std::vector<moveit_msgs::CollisionObject>& add_cos,
                                                        const std::vector<moveit_msgs::CollisionObject>& remove_cos,
                                                        const std::vector<moveit_msgs::AttachedCollisionObject>& attach_objs,
                                                        const std::vector<moveit_msgs::AttachedCollisionObject>& detach_objs);

// TODO: should replace with a more general one, quickfix is an overload
// overload for picknplace planning scene construction
void constructSubprocessPlanningScenes(moveit::core::RobotModelConstPtr moveit_model,
                                       const std::string& world_frame,
                                       const choreo_msgs::AssemblySequencePickNPlace& as_pnp,
                                       std::vector<std::vector<planning_scene::PlanningScenePtr>>& planning_scenes_subprocess);

void constructTransitionPlanningScenes(moveit::core::RobotModelConstPtr moveit_model,
                                       descartes_core::RobotModelPtr model,
                                       const std::string& world_frame,
                                       const choreo_msgs::AssemblySequencePickNPlace& as_pnp,
                                       const std::vector <choreo_msgs::UnitProcessPlan>& plans,
                                       const std::vector<std::vector<planning_scene::PlanningScenePtr>>& planning_scenes_subprocess,
                                       std::vector<std::vector<planning_scene::PlanningScenePtr>>& planning_scenes_transition);

// overload for spatial extrusion planning scene construction
void constructPlanningScenes(moveit::core::RobotModelConstPtr moveit_model,
                                     const std::vector <choreo_msgs::WireFrameCollisionObject> &wf_collision_objs,
                                     std::vector<planning_scene::PlanningScenePtr>& planning_scenes_shrinked_approach,
                                     std::vector<planning_scene::PlanningScenePtr>& planning_scenes_shrinked_depart,
                                     std::vector<planning_scene::PlanningScenePtr>& planning_scenes_full);

}

#endif //CHOREO_MPP_CONSTRUCT_PLANNING_SCENE_H_H