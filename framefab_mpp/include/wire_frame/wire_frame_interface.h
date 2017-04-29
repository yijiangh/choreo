//
// Created by yijiangh on 4/27/17.
//

#ifndef FRAMEFAB_MPP_WIRE_FRAME_INTERFACE_H
#define FRAMEFAB_MPP_WIRE_FRAME_INTERFACE_H

#include <vector>
#include <memory>

#include <wire_frame/Vec.h>

namespace framefab
{
namespace wire_frame
{

class WireFrameVertInterface;
typedef std::shared_ptr<WireFrameVertInterface> WireFrameVertInterfacePtr;
class WireFrameEdgeInterface;
typedef std::shared_ptr<WireFrameEdgeInterface> WireFrameEdgeInterfacePtr;

class WireFrameVertInterface
{
 protected:
  typedef trimesh::point point;
  typedef trimesh::vec3  Vec3f;

 public:
  WireFrameVertInterface() {}
  virtual ~WireFrameVertInterface() {}

 public:
  virtual point	Position() const = 0;
  virtual int	ID()	   const = 0;
  virtual int	Degree()   const = 0;
  virtual WireFrameEdgeInterfacePtr getNeighborEdge() const = 0;

  virtual bool isFixed() const = 0;
  virtual bool isBase()  const = 0;
  virtual bool isSubgraph() const = 0;
};

class WireFrameEdgeInterface
{
 protected:
  typedef trimesh::point point;
  typedef trimesh::vec3  Vec3f;

 public:
  WireFrameEdgeInterface() {}
  virtual ~WireFrameEdgeInterface() {}

 public:
  virtual int   ID()	   const = 0;
  virtual int   Layer() const = 0;
  virtual WireFrameVertInterfacePtr getVert() const = 0;
  virtual WireFrameEdgeInterfacePtr getPairEdge() const = 0;
  virtual WireFrameEdgeInterfacePtr getNextEdge() const = 0;

  virtual bool  isPillar()    const = 0;
  virtual bool  isCeiling()   const = 0;
  virtual bool  isSubgraph()  const = 0;

  virtual point CenterPos() const = 0;
  virtual double Length() const = 0;
};

class WireFrameInterface
{
 protected:
  typedef trimesh::point point;
  typedef trimesh::vec3  Vec3f;

 public:
  WireFrameInterface() {}
  virtual ~WireFrameInterface() {}

 public:
  virtual inline int SizeOfVertList()   const = 0;
  virtual inline int SizeOfEdgeList()   const = 0;
  virtual inline int SizeOfFixedVert()  const = 0;
  virtual inline int SizeOfBaseVert()   const = 0;
  virtual inline int SizeOfPillar()     const = 0;
  virtual inline int SizeOfCeiling()    const = 0;
  virtual inline int SizeOfLayer()      const = 0;

  // TODO: should make these query function const
//  virtual inline std::vector<WireFrameVertInterface*>* GetVertList() = 0;
//  virtual inline std::vector<WireFrameEdgeInterface*>* GetEdgeList() = 0;
  virtual inline WireFrameVertInterface* GetVert(int u)  const = 0;
  virtual inline WireFrameEdgeInterface* GetEdge(int i)  const = 0;
  virtual inline WireFrameEdgeInterface* GetNeighborEdge(int u) const = 0;

  virtual inline point		GetPosition(int u) const = 0;
  virtual inline int		GetDegree(int u)   const = 0;

  /*! @brief get endpoint u's id of input edge i
   *  @param input edge's index i
   */
  virtual inline int	GetEndu(int i) const = 0;

  /*! @brief get endpoint v's id of input edge i
   *  @param input edge's index i
   */
  virtual inline int	GetEndv(int i)  const = 0;
  virtual inline point  GetCenterPos(int i) const = 0;
  virtual inline bool   isFixed(int u)  const = 0;
  virtual inline bool   isPillar(int i) const = 0;

  virtual inline double	maxX() const = 0;
  virtual inline double	minX() const = 0;
  virtual inline double	maxY() const = 0;
  virtual inline double	minY() const = 0;
  virtual inline double	maxZ() const = 0;
  virtual inline double	minZ() const = 0;

  virtual inline double Norm(point u) const = 0;
  virtual inline double Dist(point u, point v) const = 0;
};

}// namespace wireframe
}// namespace framefab

#endif //FRAMEFAB_MPP_WIRE_FRAME_INTERFACE_H
