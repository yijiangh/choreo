//
// Created by yijiangh on 4/29/17.
//

#ifndef FRAMEFAB_MPP_WIRE_FRAME_COLLISION_OBJECTS_H
#define FRAMEFAB_MPP_WIRE_FRAME_COLLISION_OBJECTS_H

#include <vector>

#include <boost/shared_ptr.hpp>

#include <geometry_msgs/Point.h>

#include <moveit_msgs/CollisionObject.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include <wire_frame/Vec.h>
#include <wire_frame/wire_frame_line_graph.h>

namespace framefab
{
namespace wire_frame
{
typedef moveit_msgs::CollisionObject    MoveitCollisionObject;
typedef moveit_msgs::CollisionObjectPtr MoveitCollisionObjectPtr;

struct MoveitLinearMemberCollisionObjects
{
 public:
  MoveitCollisionObject start_vertex_collision;
  MoveitCollisionObject end_vertex_collision;
  MoveitCollisionObject edge_cylinder_collision;
};
typedef boost::shared_ptr<MoveitLinearMemberCollisionObjects>       MoveitLinearMemberCollisionObjectsPtr;
typedef std::vector<MoveitLinearMemberCollisionObjectsPtr>          MoveitLinearMemberCollisionObjectsList;
typedef boost::shared_ptr<MoveitLinearMemberCollisionObjectsList>   MoveitLinearMemberCollisionObjectsListPtr;

/*! @class WireFrameCollisionObjects
 *  @brief extended class for WireFrameLineGraph with Moveit Collision Objects
 */
class WireFrameCollisionObjects : public WireFrameLineGraph
{
 public:
  WireFrameCollisionObjects();
  ~WireFrameCollisionObjects() {}

 public:
  //! construct moveit_msg collision object using WF_line_graph info
  void constructCollisionObjects(
      const planning_scene_monitor::PlanningSceneMonitorConstPtr ptr_planning_scene_monitor,
      const int pwf_scale_factor,
      const double display_point_radius,
      const trimesh::point offset_vec);

  inline MoveitLinearMemberCollisionObjectsListPtr getCollisionObjectsList()
  {
    return ptr_linear_member_collision_objects_list_;
  }

  inline MoveitLinearMemberCollisionObjectsPtr getCollisionObject(int i)
  {
    return (i >= ptr_linear_member_collision_objects_list_->size() || i < 0) ? NULL :
           (*ptr_linear_member_collision_objects_list_)[i];
  }

 private:
  geometry_msgs::Point transformPoint(
      const trimesh::point target_point,
      const trimesh::point offset_vec,
      const double scale_factor);

  geometry_msgs::Pose  computeCylinderPose(
      const geometry_msgs::Point st_point,
      const geometry_msgs::Point center_point,
      const geometry_msgs::Point end_point);

 private:
  MoveitLinearMemberCollisionObjectsListPtr ptr_linear_member_collision_objects_list_;
};
typedef boost::shared_ptr<WireFrameCollisionObjects> WireFrameCollisionObjectsPtr;

}// namespace wireframe
}// namespace framefab

#endif //FRAMEFAB_MPP_WIRE_FRAME_COLLISION_OBJECTS_H
