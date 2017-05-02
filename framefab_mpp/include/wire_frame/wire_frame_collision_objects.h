//
// Created by yijiangh on 4/29/17.
//

#ifndef FRAMEFAB_MPP_WIRE_FRAME_COLLISION_OBJECTS_H
#define FRAMEFAB_MPP_WIRE_FRAME_COLLISION_OBJECTS_H

#include <vector>
#include <memory>

#include <moveit_msgs/CollisionObject.h>

#include <wire_frame/Vec.h>
#include <wire_frame/wire_frame_interface.h>
#include <wire_frame/wire_frame_line_graph.h>

namespace framefab
{
namespace wire_frame
{
typedef moveit_msgs::CollisionObjectPtr MoveitCollisionObjectPtr;

typedef std::vector<moveit_msgs::CollisionObjectPtr>                  MoveitCollisionObjectsList;
typedef std::shared_ptr<std::vector<moveit_msgs::CollisionObjectPtr>> MoveitCollisionObjectsListPtr;

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
  void constructCollisionObjects();

  inline MoveitCollisionObjectsListPtr getVertCollisionObjectsList() { return ptr_vert_collision_objects_list_; }
  inline MoveitCollisionObjectsListPtr getEdgeCollisionObjectsList() { return ptr_edge_collision_objects_list_; }

  inline MoveitCollisionObjectPtr getVertCollisionObject(int u)
  {
    return (u >= SizeOfVertList() || u < 0) ? NULL : (*ptr_vert_collision_objects_list_)[u];
  }

  inline MoveitCollisionObjectPtr getEdgeCollisionObject(int i)
  {
    return (i >= SizeOfEdgeList() || i < 0) ? NULL : (*ptr_edge_collision_objects_list_)[i];
  }

 private:
  MoveitCollisionObjectsListPtr ptr_vert_collision_objects_list_;
  MoveitCollisionObjectsListPtr ptr_edge_collision_objects_list_;
};
typedef std::shared_ptr<WireFrameCollisionObjects> WireFrameCollisionObjectsPtr;

}// namespace wireframe
}// namespace framefab

#endif //FRAMEFAB_MPP_WIRE_FRAME_COLLISION_OBJECTS_H
