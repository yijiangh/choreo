//
// Created by yijiangh on 4/9/18.
//

#ifndef FRAMEFAB_MPP_CONSTRUCT_PLANNING_SCENE_H
#define FRAMEFAB_MPP_CONSTRUCT_PLANNING_SCENE_H

#include <framefab_msgs/AssemblySequencePickNPlace.h>
#include <framefab_msgs/WireFrameCollisionObject.h>

#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/AttachedCollisionObject.h>

#include <moveit/planning_scene/planning_scene.h>
#include <descartes_core/robot_model.h>

namespace framefab_process_planning
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
void constructPlanningScenes(moveit::core::RobotModelConstPtr moveit_model,
                                     const std::string& world_frame,
                                     const framefab_msgs::AssemblySequencePickNPlace& as_pnp,
                                     std::vector<std::vector<planning_scene::PlanningScenePtr>>& planning_scenes_transition,
                                     std::vector<std::vector<planning_scene::PlanningScenePtr>>& planning_scenes_subprocess);

// overload for spatial extrusion planning scene construction
void constructPlanningScenes(moveit::core::RobotModelConstPtr moveit_model,
                                     const std::vector <framefab_msgs::WireFrameCollisionObject> &wf_collision_objs,
                                     std::vector<planning_scene::PlanningScenePtr>& planning_scenes_shrinked_approach,
                                     std::vector<planning_scene::PlanningScenePtr>& planning_scenes_shrinked_depart,
                                     std::vector<planning_scene::PlanningScenePtr>& planning_scenes_full);

}

#endif //FRAMEFAB_MPP_CONSTRUCT_PLANNING_SCENE_H_H