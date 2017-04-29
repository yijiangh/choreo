//
// Created by yijiangh on 4/29/17.
//

#ifndef FRAMEFAB_MPP_WIRE_FRAME_COLLISION_OBJECTS_H
#define FRAMEFAB_MPP_WIRE_FRAME_COLLISION_OBJECTS_H

#include <vector>
#include <memory>

#include <moveit_msgs/CollisionObject.h>

#include <wire_frame/Vec.h>
#include <wire_frame_interface.h>
#include <wire_frame_line_graph.h>

namespace framefab
{
namespace wire_frame
{

class WireFrameVertCollisionObject;
class WireFrameEdgeCollisionObjectObject;

typedef std::shared_ptr<WireFrameVertCollisionObject>                 WireFrameVertCollisionObjectPtr;
typedef std::shared_ptr<const WireFrameVertCollisionObject>           WireFrameVertCollisionObjectConstPtr;
typedef std::shared_ptr<std::vector<WireFrameVertCollisionObjectPtr>> WireFrameVertCollisionObjectsListPtr;

typedef std::shared_ptr<WireFrameEdgeCollisionObject>                 WireFrameEdgeCollisionObjectPtr;
typedef std::shared_ptr<const WireFrameEdgeCollisionObject>           WireFrameEdgeCollisionObjectConstPtr;
typedef std::shared_ptr<std::vector<WireFrameEdgeCollisionObjectPtr>> WireFrameEdgeCollisionObjectsListPtr;

class WireFrameVertCollisionObject : public WireFrameVertInterface
{
 protected:
  typedef trimesh::point point;
  typedef trimesh::vec3  Vec3f;

 public:
  WireFrameVertInterface() {}
  virtual ~WireFrameVertInterface() {}

 public:
  point		Position() { return ptr_wire_frame_vert_->Position(); }
  int		ID()	   { return ptr_wire_frame_vert_->ID(); }
  int		Degree()   { return ptr_wire_frame_vert_->Degree(); }
  WireFrameEdgeInterfacePtr getNeighborEdge() const { return ptr_moveit_collision_object_neighbor_edge_; }

  bool isFixed() const { return ptr_wire_frame_vert_->isFixed(); }
  bool isBase()  const { return ptr_wire_frame_vert_->isBase(); }
  bool isSubgraph() const { return ptr_wire_frame_vert_->isSubgraph(); }

 private:
  WFVertConstPtr                  ptr_wire_frame_vert_;
  moveit_msgs::CollisionObjectPtr ptr_moveit_collision_object_node_;
  WireFrameEdgeCollisionObjectPtr ptr_moveit_collision_object_neighbor_edge_;
};

class WireFrameEdgeCollisionObject : public WireFrameEdgeInterface
{
 protected:
  typedef trimesh::point point;
  typedef trimesh::vec3  Vec3f;

 public:
  WireFrameEdgeInterface() {}
  virtual ~WireFrameEdgeInterface() {}

 public:
  int   ID()	const { return ptr_wire_frame_edge_->ID(); }
  int   Layer() const { return ptr_wire_frame_edge_->Layer(); }
  bool  isPillar()    const { return ptr_wire_frame_edge_->isPillar(); }
  bool  isCeiling()   const { return ptr_wire_frame_edge_->isCeiling(); }
  bool  isSubgraph()  const { return ptr_wire_frame_edge_->isSubgraph(); }

  point CenterPos() const { return ptr_wire_frame_edge_->CenterPos(); }
  double Length()   const { return ptr_wire_frame_edge_->Length(); }

 private:
  WFEdgeConstPtr                  ptr_wire_frame_edge_;
  moveit_msgs::CollisionObjectPtr ptr_moveit_collision_object_edge_;
};

class WireFrameCollisionObjects : public WireFrameInterface
{
 public:
  WireFrameCollisionObjects();
  ~WireFrameCollisionObjects() {}

 public:
  /*! @brief constructor from input WireFrameLineGraph
   *  @param wire frame line graph shared pointer
   */
  void loadFromLineGraph(WireFrameLineGraphPtr ptr_wire_frame_line_graph);

 public:
  inline int SizeOfVertList()   const { return ptr_vert_collision_obj_list_->size(); }
  inline int SizeOfEdgeList()   const { return ptr_edge_collision_obj_list_->size(); }
  inline int SizeOfFixedVert()  const { return ptr_wireframe_linegraph_->SizeOfFixedVert(); }
  inline int SizeOfBaseVert()   const { return ptr_wireframe_linegraph_->SizeOfBaseVert(); }
  inline int SizeOfPillar()     const { return ptr_wireframe_linegraph_->SizeOfPillar(); }
  inline int SizeOfCeiling()    const { return ptr_wireframe_linegraph_->SizeOfCeiling(); }
  inline int SizeOfLayer()      const { return ptr_wireframe_linegraph_->SizeOfLayer(); }

  // TODO: should make these query function const
  inline WireFrameVertCollisionObjectsListPtr GetVertList() { return ptr_vert_collision_obj_list_; }
  inline WireFrameEdgeCollisionObjectsListPtr GetEdgeList() { return ptr_edge_collision_obj_list_; }
  inline WireFrameVertInterface* GetVert(int u) const
  {
    return (u >= SizeOfVertList() || u < 0) ? NULL : (*ptr_vert_collision_obj_list_)[u];
  }

  inline WireFrameEdgeInterface* GetEdge(int i) const
  {
    return (i >= SizeOfEdgeList() || i < 0) ? NULL : (*ptr_edge_collision_obj_list_)[i];
  }

  inline WireFrameEdgeInterface* GetNeighborEdge(int u) const
  {
    return (u >= SizeOfVertList() || u < 0) ? NULL : (*ptr_vert_collision_obj_list_)[u]->pedge_;
  }

  inline point	GetPosition(int u) const { return ptr_wireframe_linegraph_->GetPosition(u); }
  inline int	GetDegree(int u)   const { return ptr_wireframe_linegraph_->GetDegree(u); }

  /*! @brief get endpoint u's id of input edge i
   *  @param input edge's index i
   */
  inline int	GetEndu(int i) const { return ptr_wireframe_linegraph_->GetEndu(i); }

  /*! @brief get endpoint v's id of input edge i
   *  @param input edge's index i
   */
  inline int	GetEndv(int i)      const { return ptr_wireframe_linegraph_->GetEndv(i); }

  inline point  GetCenterPos(int i) const { return ptr_wireframe_linegraph_->GetCenterPos(i); }
  inline bool   isFixed(int u)      const { return ptr_wireframe_linegraph_->isFixed(u); }
  inline bool   isPillar(int i)     const { return ptr_wireframe_linegraph_->isPillar(); }

  inline double	maxX() const { return ptr_wireframe_linegraph_->maxX(); }
  inline double	minX() const { return ptr_wireframe_linegraph_->minX(); }
  inline double	maxY() const { return ptr_wireframe_linegraph_->maxY(); }
  inline double	minY() const { return ptr_wireframe_linegraph_->minY(); }
  inline double	maxZ() const { return ptr_wireframe_linegraph_->maxZ(); }
  inline double	minZ() const { return ptr_wireframe_linegraph_->minZ(); }

  inline double Norm(point u) { return ptr_wireframe_linegraph_->Norm(u); }
  inline double Dist(point u, point v) const { return ptr_wireframe_linegraph_->Dist(u, v); }

 private:
  WireFrameLineGraphPtr ptr_wireframe_linegraph_;

  WireFrameVertCollisionObjectsListPtr ptr_vert_collision_obj_list_;
  WireFrameEdgeCollisionObjectsListPtr ptr_edge_collision_obj_list_;
};

typedef std::shared_ptr<WireFrameCollisionObjects> WireFrameCollisionObjectsPtr;
typedef std::shared_ptr<const WireFrameCollisionObjects> WireFrameCollisionObjectsConstPtr;
}// namespace wireframe
}// namespace framefab

#endif //FRAMEFAB_MPP_WIRE_FRAME_COLLISION_OBJECTS_H
