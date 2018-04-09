//
// Created by yijiangh on 4/9/18.
//

#include "construct_planning_scene.h"

#include <choreo_geometry_conversion_helpers/choreo_geometry_conversion_helpers.h>

#include <shape_msgs/Mesh.h>

#include <moveit_msgs/PlanningSceneComponents.h>
#include <moveit_msgs/ApplyPlanningScene.h>
#include <moveit_msgs/GetPlanningScene.h>

const static std::string PICKNPLACE_EEF_NAME = "mit_arch_suction_gripper";

const static std::string GET_PLANNING_SCENE_SERVICE = "get_planning_scene";

// TODO: replace this hardcoded scale vector with parm input!!
const static Eigen::Vector3d MESH_SCALE_VECTOR = Eigen::Vector3d(0.001, 0.001, 0.001);

namespace {

void setEmptyPoseMsg(geometry_msgs::Pose& pose)
{
  pose.position.x = 0.0;
  pose.position.y = 0.0;
  pose.position.z = 0.0;
  pose.orientation.w= 0.0;
  pose.orientation.x= 0.0;
  pose.orientation.y= 0.0;
  pose.orientation.z= 0.0;
}

}
namespace framefab_process_planning
{

void constructPlanningScene(const planning_scene::PlanningScenePtr base_scene,
                            const std::vector <moveit_msgs::CollisionObject> &add_cos,
                            const std::vector <moveit_msgs::CollisionObject> &remove_cos,
                            const std::vector <moveit_msgs::AttachedCollisionObject> &attach_objs,
                            const std::vector <moveit_msgs::AttachedCollisionObject> &detach_objs,
                            planning_scene::PlanningScenePtr s)
{
  s = base_scene->diff();

  for(auto add_co : add_cos)
  {
    add_co.operation = moveit_msgs::CollisionObject::ADD;
    assert(s->processCollisionObjectMsg(add_co));
  }

  for(auto remove_co : remove_cos)
  {
    remove_co.operation = moveit_msgs::CollisionObject::REMOVE;
    assert(s->processCollisionObjectMsg(remove_co));
  }

  for(auto ato : attach_objs)
  {
    ato.object.operation = moveit_msgs::CollisionObject::ADD;
    assert(s->processAttachedCollisionObjectMsg(ato));
  }

  for(auto rto : detach_objs)
  {
    rto.object.operation = moveit_msgs::CollisionObject::REMOVE;
    assert(s->processAttachedCollisionObjectMsg(rto));
  }
}

planning_scene::PlanningScenePtr constructPlanningScene(const planning_scene::PlanningScenePtr base_scene,
                                                        const std::vector <moveit_msgs::CollisionObject> &add_cos,
                                                        const std::vector <moveit_msgs::CollisionObject> &remove_cos,
                                                        const std::vector <moveit_msgs::AttachedCollisionObject> &attach_objs,
                                                        const std::vector <moveit_msgs::AttachedCollisionObject> &detach_objs)
{
  planning_scene::PlanningScenePtr scene = base_scene->diff();
  constructPlanningScene(base_scene, add_cos, remove_cos, attach_objs, detach_objs, scene);
  return scene;
}

void constructPlanningScenes(moveit::core::RobotModelConstPtr moveit_model,
                             const framefab_msgs::AssemblySequencePickNPlace as_pnp,
                             const std::string& world_frame,
                             std::vector<planning_scene::PlanningScenePtr>& planning_scenes_transition2pick,
                             std::vector<planning_scene::PlanningScenePtr>& planning_scenes_pick,
                             std::vector<collision_detection::AllowedCollisionMatrix>& pick_acms,
                             std::vector<planning_scene::PlanningScenePtr>& planning_scenes_transition2place,
                             std::vector<planning_scene::PlanningScenePtr>& planning_scenes_place,
                             std::vector<collision_detection::AllowedCollisionMatrix>& place_acms)
{
  using namespace choreo_geometry_conversion_helpers;

  const auto build_scenes_start = ros::Time::now();

  // get current planning scene as base scene
  ros::NodeHandle nh;
  auto planning_scene_client = nh.serviceClient<moveit_msgs::GetPlanningScene>(GET_PLANNING_SCENE_SERVICE);

  if (!planning_scene_client.waitForExistence())
  {
    ROS_ERROR_STREAM("[Process Planning] cannot connect with get planning scene server...");
  }

  moveit_msgs::GetPlanningScene srv;
  srv.request.components.components =
      moveit_msgs::PlanningSceneComponents::ROBOT_STATE
          | moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_GEOMETRY;

  if (!planning_scene_client.call(srv))
  {
    ROS_ERROR_STREAM("[Process Planning] Failed to fetch planning scene srv!");
  }

  planning_scene::PlanningScenePtr root_scene(new planning_scene::PlanningScene(moveit_model));
  root_scene->getCurrentStateNonConst().setToDefaultValues();
  root_scene->getCurrentStateNonConst().update();
  root_scene->setPlanningSceneDiffMsg(srv.response.scene);

  // parse all the collision objects, they are naturally named after id number
  std::vector <moveit_msgs::CollisionObject> pick_cos;
  std::vector <moveit_msgs::CollisionObject> place_cos;

  std::map <std::string, moveit_msgs::CollisionObject> pick_support_surfaces;
  std::map <std::string, moveit_msgs::CollisionObject> place_support_surfaces;

  pick_acms.clear();
  place_acms.clear();
  planning_scenes_transition2pick.clear();
  planning_scenes_pick.clear();
  planning_scenes_transition2place.clear();
  planning_scenes_place.clear();

  // TODO: replace this hard-coded eef name!!
  const robot_model::JointModelGroup *eef = moveit_model->getEndEffector(PICKNPLACE_EEF_NAME);
  assert(eef);
  const std::string &ik_link = eef->getEndEffectorParentGroup().second;

  // use empty pose for all objs, as the positions are built in the mesh itself (using global base frame)
  geometry_msgs::Pose empty_pose;
  setEmptyPoseMsg(empty_pose);

  // TODO: maybe should use file name explicitly here, to be safer
  for(const std::string &pn : as_pnp.pick_support_surface_file_names)
  {
    pick_support_surfaces[pn] = savedSTLToCollisionObjectMsg(
        as_pnp.file_path + pn,
        MESH_SCALE_VECTOR,
        world_frame,
        empty_pose);
  }

  for(const std::string &pn : as_pnp.place_support_surface_file_names)
  {
    place_support_surfaces[pn] = savedSTLToCollisionObjectMsg(
        as_pnp.file_path + pn,
        MESH_SCALE_VECTOR,
        world_frame,
        empty_pose);
  }

  for(const auto &se : as_pnp.sequenced_elements)
  {
    // construct pick element collision mesh
    pick_cos.push_back(
        savedSTLToCollisionObjectMsg(
            se.file_path + se.pick_element_geometry_file_name,
            MESH_SCALE_VECTOR,
            world_frame,
            empty_pose));

    place_cos.push_back(
        savedSTLToCollisionObjectMsg(
            se.file_path + se.place_element_geometry_file_name,
            MESH_SCALE_VECTOR,
            world_frame,
            empty_pose));
  }

  assert(pick_cos.size() == as_pnp.sequenced_elements.size());
  assert(place_cos.size() == as_pnp.sequenced_elements.size());

  // construct planning scene
  for (const auto &se : as_pnp.sequenced_elements)
  {
    // transition 2 pick

    // pick: approach - pick - depart

    // transition 2 place

    // place: approach - place (drop) - depart

    // Allowed Collision Object
    auto pick_acm = root_scene->getAllowedCollisionMatrixNonConst();
    auto place_acm = root_scene->getAllowedCollisionMatrixNonConst();

    // during picking, end effector is allowed to touch the pick block
    // pick block is allowed to touch neighboring blocks and support surface
    pick_acm.setEntry(ik_link, se.pick_element_geometry_file_name, true);

    for(const auto& id : se.pick_contact_ngh_ids)
    {
      pick_acm.setEntry(se.pick_element_geometry_file_name, pick_cos[id].id, true);
    }

    for(const auto& id : se.pick_support_surface_file_names)
    {
      pick_acm.setEntry(se.pick_element_geometry_file_name, pick_support_surfaces[id].id, true);
    }

    pick_acms.push_back(pick_acm);

    // during placing, end effector is allowed to touch the place block
    // place block is allowed to touch neighboring blocks and support surface
    place_acm.setEntry(ik_link, se.place_element_geometry_file_name, true);

    for(const auto& id : se.place_contact_ngh_ids)
    {
      place_acm.setEntry(se.place_element_geometry_file_name, place_cos[id].id, true);
    }

    for(const auto& id : se.place_support_surface_file_names)
    {
      place_acm.setEntry(se.place_element_geometry_file_name, place_support_surfaces[id].id, true);
    }

    place_acms.push_back(place_acm);
  }

  const auto build_scenes_end = ros::Time::now();
  ROS_INFO_STREAM(
      "[Process Planning] constructing planning scenes took " << (build_scenes_end - build_scenes_start).toSec()
                                                              << " seconds.");
}

void constructPlanningScenes(moveit::core::RobotModelConstPtr moveit_model,
                             const std::vector <framefab_msgs::WireFrameCollisionObject> &wf_collision_objs,
                             std::vector <planning_scene::PlanningScenePtr> &planning_scenes_shrinked_approach,
                             std::vector <planning_scene::PlanningScenePtr> &planning_scenes_shrinked_depart,
                             std::vector <planning_scene::PlanningScenePtr> &planning_scenes_full)
{
  const auto build_scenes_start = ros::Time::now();

  planning_scenes_shrinked_approach.reserve(wf_collision_objs.size());
  planning_scenes_shrinked_depart.reserve(wf_collision_objs.size());
  planning_scenes_full.reserve(wf_collision_objs.size());

  ros::NodeHandle nh;
  auto planning_scene_client = nh.serviceClient<moveit_msgs::GetPlanningScene>(GET_PLANNING_SCENE_SERVICE);

  if (!planning_scene_client.waitForExistence())
  {
    ROS_ERROR_STREAM("[Process Planning] cannot connect with get planning scene server...");
  }

  moveit_msgs::GetPlanningScene srv;
  srv.request.components.components =
      moveit_msgs::PlanningSceneComponents::ROBOT_STATE
          | moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_GEOMETRY;

  if (!planning_scene_client.call(srv))
  {
    ROS_ERROR_STREAM("[Process Planning] Failed to fetch planning scene srv!");
  }

  planning_scene::PlanningScenePtr root_scene(new planning_scene::PlanningScene(moveit_model));
  root_scene->getCurrentStateNonConst().setToDefaultValues();
  root_scene->getCurrentStateNonConst().update();
  root_scene->setPlanningSceneDiffMsg(srv.response.scene);

  // start with no element constructed in the scene
  planning_scenes_shrinked_approach.push_back(root_scene);
  planning_scenes_shrinked_depart.push_back(root_scene);
  planning_scenes_full.push_back(root_scene);

  for (std::size_t i = 1; i < wf_collision_objs.size(); ++i) // We use all but the last collision object
  {
    auto last_scene_shrinked = planning_scenes_shrinked_approach.back();
    auto child_shrinked = last_scene_shrinked->diff();

    auto c_list = wf_collision_objs[i].recovered_last_neighbor_objs;
    c_list.push_back(wf_collision_objs[i].last_full_obj);

    const auto &shrinked_neighbor_objs = wf_collision_objs[i].shrinked_neighbor_objs;
    c_list.insert(c_list.begin(), shrinked_neighbor_objs.begin(), shrinked_neighbor_objs.end());

    for (const auto &c_obj : c_list)
    {
      if (!child_shrinked->processCollisionObjectMsg(c_obj))
      {
        ROS_WARN("[Process Planning] Failed to process shrinked collision object");
      }
    }

    auto child_shrinked_depart = child_shrinked->diff();
    if (!child_shrinked_depart->processCollisionObjectMsg(wf_collision_objs[i].both_side_shrinked_obj))
    {
      ROS_WARN("[Process Planning] Failed to process shrinked collision object");
    }

    // push in partial_collision_geometry_planning_scene
    planning_scenes_shrinked_approach.push_back(child_shrinked);
    planning_scenes_shrinked_depart.push_back(child_shrinked_depart);

    // get diff as child
    // restore changed element back to full geometry
    // push in full_collision_geometry_planning_scene
    auto last_scene_full = planning_scenes_full.back();
    auto child_full = last_scene_full->diff();

    // TODO: temp fix
    auto inflated_full_obj = wf_collision_objs[i - 1].full_obj;
//    inflated_full_obj.primitives[0].dimensions[1] += 0.0;

    if (!child_full->processCollisionObjectMsg(inflated_full_obj))
    {
      ROS_WARN("[Process Planning] Failed to process full collision object");
    }

    // push in full_scene[i]
    planning_scenes_full.push_back(child_full);
  }

  const auto build_scenes_end = ros::Time::now();
  ROS_INFO_STREAM(
      "[Process Planning] constructing planning scenes took " << (build_scenes_end - build_scenes_start).toSec()
                                                              << " seconds.");
}

}