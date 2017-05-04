/*
* ==========================================================================
*		This file is part of the implementation of
*
*		<FrameFab: Robotic Fabrication of Frame Shapes>
*		Yijiang Huang, Juyong Zhang, Xin Hu, Guoxian Song, Zhongyuan Liu, Lei Yu, Ligang Liu
*		In ACM Transactions on Graphics (Proc. SIGGRAPH Asia 2016)
----------------------------------------------------------------------------
*		class:	WireFrame
*
*		Description: WireFrame is the basic data structure to store original frame shape in
*		FrameFab.
*
*		Version:  2.0
*		Created: Oct/10/2015
*		Updated: Aug/24/2016
*
*		Author:  Xin Hu, Yijiang Huang, Guoxian Song
*		Company:  GCL@USTC
----------------------------------------------------------------------------
*		Copyright (C) 2016  Yijiang Huang, Xin Hu, Guoxian Song, Juyong Zhang
*		and Ligang Liu.
*
*		FrameFab is free software: you can redistribute it and/or modify
*		it under the terms of the GNU General Public License as published by
*		the Free Software Foundation, either version 3 of the License, or
*		(at your option) any later version.
*
*		FrameFab is distributed in the hope that it will be useful,
*		but WITHOUT ANY WARRANTY; without even the implied warranty of
*		MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*		GNU General Public License for more details.
*
*		You should have received a copy of the GNU General Public License
*		along with FrameFab.  If not, see <http://www.gnu.org/licenses/>.
* ==========================================================================
*/

#ifndef FRAMEFAB_MPP_WIRE_FRAME_LINE_GRAPH_H
#define FRAMEFAB_MPP_WIRE_FRAME_LINE_GRAPH_H

#include <assert.h>
#include <vector>

#include <boost/shared_ptr.hpp>

#include <wire_frame/Vec.h>

#include <ros/ros.h>

namespace framefab
{
namespace wire_frame
{
typedef trimesh::point point;
typedef trimesh::vec3  Vec3f;

class WF_vert;
typedef boost::shared_ptr<WF_vert> WFVertPtr;
typedef boost::shared_ptr<const WF_vert> WFVertConstPtr;

class WF_edge;
typedef boost::shared_ptr<WF_edge> WFEdgePtr;
typedef boost::shared_ptr<const WF_edge> WFEdgeConstPtr;

class WF_vert
{

 public:
  WF_vert()
      : pedge_(NULL), id_(0), degree_(0),
        b_fixed_(false), b_base_(false), b_subg_(false) {}
  WF_vert(Vec3f p)
      : pedge_(NULL), position_(p), render_pos_(p),
        id_(0), degree_(0),
        b_fixed_(false), b_base_(false), b_subg_(false) {}
  WF_vert(double x, double y, double z)
      : pedge_(NULL), position_(point(x, y, z)), render_pos_(point(x, y, z)),
        id_(0), degree_(0),
        b_fixed_(false), b_base_(false), b_subg_(false) {}
  ~WF_vert() {}

 public:
  point Position()  const { return position_; }
  point RenderPos() const { return render_pos_; }
  int ID()      const { return id_; }
  int Degree()  const { return degree_; }

  bool isFixed()    const { return b_fixed_; }
  bool isBase()     const { return b_base_; }
  bool isSubgraph() const { return b_subg_; }

  void SetPosition(point p) { position_ = p; }
  void SetPosition(double x, double y, double z) { position_ = point(x, y, z); }
  void SetRenderPos(point p) { render_pos_ = p; }
  void SetRenderPos(double x, double y, double z) { render_pos_ = point(x, y, z); }
  void SetID(int id) { id_ = id; }

  void IncreaseDegree() { degree_++; }

  void SetFixed(bool b_fixed) { b_fixed_ = b_fixed; }
  void SetBase(bool b_base) { b_base_ = b_base; }
  void SetSubgraph(bool b_subg) { b_subg_ = b_subg; }

 public:
  WF_edge   *pedge_;

 protected:
  point position_;
  point render_pos_;

  int id_;
  int degree_;

  bool b_base_;
  bool b_fixed_;
  bool b_subg_;
};

class WF_edge
{
 public:
  WF_edge()
      : pvert_(NULL), pnext_(NULL), ppair_(NULL),
        id_(0), layer_(0), b_pillar_(false), b_ceiling_(false), b_subg_(false) {}
  ~WF_edge() {}

 public:
  int ID()          const { return id_; }
  int Layer()       const { return layer_; }

  bool isPillar()   const { return b_pillar_; }
  bool isCeiling()  const { return b_ceiling_; }
  bool isSubgraph() const { return b_subg_; }

  void SetID(int id) { id_ = id; }
  void SetLayer(int layer) { layer_ = layer; }
  void SetPillar(bool b_pillar) { b_pillar_ = b_pillar; }
  void SetCeiling(bool b_ceiling) { b_ceiling_ = b_ceiling; }
  void SetSubgraph(bool b_subg) { b_subg_ = b_subg; }

  void setVert() {}
  void setPairEdge() {}
  void setNextEdge() {}

  point CenterPos() const
  {
    point u = pvert_->Position();
    point v = ppair_->pvert_->Position();
    return point((u.x() + v.x()) / 2, (u.y() + v.y()) / 2, (u.z() + v.z()) / 2);
  }

  double Length() const
  {
    point u = pvert_->Position();
    point v = ppair_->pvert_->Position();
    double dx = u.x() - v.x();
    double dy = u.y() - v.y();
    double dz = u.z() - v.z();
    return sqrt(dx * dx + dy * dy + dz * dz);
  }

 public:
  WF_vert *pvert_;
  WF_edge *pnext_;
  WF_edge *ppair_;

 protected:
  int id_;
  int layer_;
  bool b_pillar_;
  bool b_ceiling_;
  bool b_subg_;
};

class WF_face
{
 public:
  WF_face() { bound_points_ = new std::vector<WF_vert*>; }
  ~WF_face() { delete bound_points_; }

 public:
  std::vector<WF_vert*> *bound_points_;
};

class WireFrameLineGraph
{
 public:
  WireFrameLineGraph();
  WireFrameLineGraph(std::vector<int>* bound);
  ~WireFrameLineGraph();

 public:
  void LoadFromOBJ(const char *path);
  void LoadFromPWF(const char *path);
  void WriteToOBJ(const char *path);
  void WriteToPWF(
      bool bVert, bool bLine,
      bool bPillar, bool bCeiling,
      bool bCut, int min_layer, int max_layer,
      const char *path
  );

  void ImportFrom3DD(const char *path);

  void ExportSubgraph(const char *path);
  void ExportPoints(int min_layer, int max_layer, const char *path);
  void ExportLines(int min_layer, int max_layer, const char *path);

  WF_vert* InsertVertex(const Vec3f p);
  WF_edge* InsertEdge(WF_vert *u, WF_vert *v);
  WF_edge* InsertOneWayEdge(WF_vert *u, WF_vert *v);

  void Unify();
  point Unify(Vec3f p);

  void SimplifyFrame();
  void ProjectBound(double len);
  void ModifyProjection(double len);
  void MakeBase(std::vector<WF_vert *> &base_v);
  void MakeCeiling(std::vector<WF_edge *> &bound_e);
  void MakeSubGraph(std::vector<WF_edge *> &subg_e);

  inline int SizeOfVertList()  const { return pvert_list_->size(); }
  inline int SizeOfEdgeList()  const { return pedge_list_->size(); }
  inline int SizeOfFixedVert() const { return fixed_vert_; }
  inline int SizeOfBaseVert()  const { return base_vert_; }
  inline int SizeOfPillar()    const { return pillar_size_; }
  inline int SizeOfCeiling()   const { return ceiling_size_; }
  inline int SizeOfLayer()     const { return layer_size_; }

  // TODO: should make these query function const
  inline std::vector<WF_vert*>* GetVertList() { return pvert_list_; }
  inline std::vector<WF_edge*>* GetEdgeList() { return pedge_list_; }
  inline WF_vert* GetVert(int u) const
  {
    return (u >= SizeOfVertList() || u < 0) ? NULL : (*pvert_list_)[u];
  }

  inline WF_edge* GetEdge(int i) const
  {
    return (i >= SizeOfEdgeList() || i < 0) ? NULL : (*pedge_list_)[i];
  }

  inline WF_edge* GetNeighborEdge(int u) const
  {
    return (u >= SizeOfVertList() || u < 0) ? NULL : (*pvert_list_)[u]->pedge_;
  }

  inline point GetPosition(int u) const
  {
    assert(u < SizeOfVertList() && u >= 0);
    return ((*pvert_list_)[u]->Position());
  }
  inline int GetDegree(int u) const
  {
    assert(u < SizeOfVertList() && u >= 0);
    return ((*pvert_list_)[u]->Degree());
  }

  inline int GetEndu(int i) const
  {
    assert(i < SizeOfEdgeList() && i >= 0);
    return ((*pedge_list_)[i]->ppair_->pvert_->ID());
  }
  inline int GetEndv(int i) const
  {
    assert(i < SizeOfEdgeList() && i >= 0);
    return ((*pedge_list_)[i]->pvert_->ID());
  }
  inline point GetCenterPos(int i) const
  {
    assert(i < SizeOfEdgeList() && i >= 0);
    return ((*pedge_list_)[i]->CenterPos());
  }

  inline bool isFixed(int u) const
  {
    assert(u < SizeOfVertList() && u >= 0);
    return ((*pvert_list_)[u]->isFixed());
  }
  inline bool isPillar(int i) const
  {
    assert(i < SizeOfEdgeList() && i >= 0);
    return ((*pedge_list_)[i]->isPillar());
  }

  inline double maxX() const { return maxx_; }
  inline double minX() const { return minx_; }
  inline double maxY() const { return maxy_; }
  inline double minY() const { return miny_; }
  inline double maxZ() const { return maxz_; }
  inline double minZ() const { return minz_; }

  inline double Norm(point u) const
  {
    return sqrt(u.x() * u.x() + u.y() * u.y() + u.z() * u.z());
  }

  inline double Dist(point u, point v) const
  {
    double dx = u.x() - v.x();
    double dy = u.y() - v.y();
    double dz = u.z() - v.z();
    return sqrt(dx * dx + dy * dy + dz * dz);
  }

  inline point CrossProduct(point u, point v) const
  {
    return point(u.y() * v.z() - u.z() * v.y(), u.z() * v.x() - u.x() * v.z(),
                 u.x() * v.y() - u.y() * v.x());
  }

  inline double ArcHeight(point u, point v1, point v2) const
  {
    point alpha = u - v1;
    point beta = v2 - v1;

    return Norm(CrossProduct(alpha, beta)) / Norm(beta);
  }

 protected:
  // TODO: replace these pointers using smart_ptr
  std::vector<WF_vert*>* pvert_list_;
  std::vector<WF_edge*>* pedge_list_;

  int fixed_vert_;
  int base_vert_;
  int pillar_size_;
  int ceiling_size_;
  int layer_size_;

  double maxx_;
  double maxy_;
  double maxz_;
  double minx_;
  double miny_;
  double minz_;

  Vec3f center_pos_;
  float scaleV_;
  double unify_size_;
  double delta_tol_;
};
typedef boost::shared_ptr<WireFrameLineGraph> WireFrameLineGraphPtr;
typedef boost::shared_ptr<const WireFrameLineGraph> WireFrameLineGraphConstPtr;
}// namespace wireframe
}// namespace framefab

#endif //FRAMEFAB_MPP_WIRE_FRAME_LINE_GRAPH_H