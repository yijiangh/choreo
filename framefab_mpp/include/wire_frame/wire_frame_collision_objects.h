//
// Created by yijiangh on 4/29/17.
//

#ifndef FRAMEFAB_MPP_WIRE_FRAME_COLLISION_OBJECTS_H
#define FRAMEFAB_MPP_WIRE_FRAME_COLLISION_OBJECTS_H

#include <vector>

#include <boost/shared_ptr.hpp>

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
  MoveitCollisionObjectPtr start_vertex_collision;
  MoveitCollisionObjectPtr end_vertex_collision;
  MoveitCollisionObjectPtr edge_cylinder_collision;
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
      const int pwf_scale_factor, const double display_point_radius);

  inline MoveitLinearMemberCollisionObjectsListPtr getVertCollisionObjectsList()
    { return ptr_vert_collision_objects_list_; }
  inline MoveitLinearMemberCollisionObjectsListPtr getEdgeCollisionObjectsList()
    { return ptr_edge_collision_objects_list_; }

  inline MoveitLinearMemberCollisionObjectsPtr getVertCollisionObject(int u)
  {
    return (u >= SizeOfVertList() || u < 0) ? NULL : (*ptr_vert_collision_objects_list_)[u];
  }

  inline MoveitLinearMemberCollisionObjectsPtr getEdgeCollisionObject(int i)
  {
    return (i >= SizeOfEdgeList() || i < 0) ? NULL : (*ptr_edge_collision_objects_list_)[i];
  }

 private:
  MoveitLinearMemberCollisionObjectsListPtr ptr_vert_collision_objects_list_;
  MoveitLinearMemberCollisionObjectsListPtr ptr_edge_collision_objects_list_;
};
typedef boost::shared_ptr<WireFrameCollisionObjects> WireFrameCollisionObjectsPtr;

}// namespace wireframe
}// namespace framefab

#endif //FRAMEFAB_MPP_WIRE_FRAME_COLLISION_OBJECTS_H
