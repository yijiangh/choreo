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

#pragma once

#include <assert.h>
#include <cmath>
#include <string.h>

#include "framefab_task_sequence_planner/utils/Vec.h"

using namespace std;
using trimesh::vec;
using trimesh::point;

typedef trimesh::point point;
typedef trimesh::vec3  Vec3f;
typedef trimesh::vec4  Vec4f;

class WF_vert;
class WF_edge;

class WF_vert
{
 public:
  WF_vert()
		  : pedge_(NULL), id_(0), degree_(0),
			b_fixed_(false), b_base_(false), b_subg_(false)
  {}
  WF_vert(Vec3f p)
		  : pedge_(NULL), position_(p), render_pos_(p),
			id_(0), degree_(0),
			b_fixed_(false), b_base_(false), b_subg_(false)
  {}
  WF_vert(double x, double y, double z)
		  : pedge_(NULL), position_(point(x, y, z)), render_pos_(point(x, y, z)),
			id_(0), degree_(0),
			b_fixed_(false), b_base_(false), b_subg_(false)
  {}
  ~WF_vert(){}

 public:
  point		Position()		{ return position_; }
  point		RenderPos()		{ return render_pos_; }
  int		ID()			{ return id_; }
  int		Degree()		{ return degree_; }

  bool		isFixed()		{ return b_fixed_; }
  bool		isBase()		{ return b_base_; }
  bool		isSubgraph()	{ return b_subg_; }

  void		SetPosition(point p)						{ position_ = p; }
  void		SetPosition(double x, double y, double z)	{ position_ = point(x, y, z); }
  void		SetRenderPos(point p)						{ render_pos_ = p; }
  void		SetRenderPos(double x, double y, double z)	{ render_pos_ = point(x, y, z); }
  void		SetID(int id)								{ id_ = id; }
  void		IncreaseDegree()							{ degree_++; }

  void		SetFixed(bool b_fixed)	 { b_fixed_ = b_fixed; }
  void		SetBase(bool b_base)	 { b_base_ = b_base; }
  void		SetSubgraph(bool b_subg) { b_subg_ = b_subg; }

 public:
  WF_edge		*pedge_;

 private:
  point		position_;
  point		render_pos_;

  int			id_;
  int			degree_;

  bool		b_base_;
  bool		b_fixed_;
  bool		b_subg_;
};


class WF_edge
{
 public:
  WF_edge()
		  :pvert_(NULL), pnext_(NULL), ppair_(NULL),
		   id_(0), layer_(0), b_pillar_(false), b_ceiling_(false), b_subg_(false)
  {}
  ~WF_edge(){}

 public:
  int ID()			{ return id_; }
  int Layer()			{ return layer_; }
  bool isPillar()		{ return b_pillar_; }
  bool isCeiling()		{ return b_ceiling_; }
  bool isSubgraph()	{ return b_subg_; }

  void SetID(int id)				{ id_ = id; }
  void SetLayer(int layer)			{ layer_ = layer; }
  void SetPillar(bool b_pillar)	{ b_pillar_ = b_pillar; }
  void SetCeiling(bool b_ceiling)	{ b_ceiling_ = b_ceiling; }
  void SetSubgraph(bool b_subg)	{ b_subg_ = b_subg; }

  point CenterPos()
  {
	  point u = pvert_->Position();
	  point v = ppair_->pvert_->Position();
	  return point((u.x() + v.x()) / 2, (u.y() + v.y()) / 2, (u.z() + v.z()) / 2);
  }

  double Length()
  {
	  point u = pvert_->Position();
	  point v = ppair_->pvert_->Position();
	  double dx = u.x() - v.x();
	  double dy = u.y() - v.y();
	  double dz = u.z() - v.z();
	  return sqrt(dx*dx + dy*dy + dz*dz);
  }

 public:
  WF_vert		*pvert_;
  WF_edge		*pnext_;
  WF_edge		*ppair_;

 private:
  int			id_;
  int			layer_;
  bool		b_pillar_;
  bool		b_ceiling_;
  bool		b_subg_;
};


class WF_face
{
 public:
  WF_face()	{ bound_points_ = new vector<WF_vert*>; }
  ~WF_face()	{ delete bound_points_; }

 public:
  vector<WF_vert*>	*bound_points_;
};


class WireFrame
{
 public:
  WireFrame();
  WireFrame(vector<int> *bound);
  ~WireFrame();

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
  void MakeBase(vector<WF_vert*> &base_v);
  void MakeCeiling(vector<WF_edge*> &bound_e);
  void MakeSubGraph(vector<WF_edge*> &subg_e);

  inline int SizeOfVertList() { return pvert_list_->size(); }
  inline int SizeOfEdgeList() { return pedge_list_->size(); }
  inline int SizeOfFixedVert() { return fixed_vert_; }
  inline int SizeOfBaseVert() { return base_vert_; }
  inline int SizeOfPillar() { return pillar_size_; }
  inline int SizeOfCeiling() { return ceiling_size_; }
  inline int SizeOfLayer() { return layer_size_; }

  inline vector<WF_vert*> *GetVertList() { return pvert_list_; }
  inline vector<WF_edge*> *GetEdgeList() { return pedge_list_; }
  inline WF_vert *GetVert(int u) { return (u >= SizeOfVertList() || u < 0) ? NULL : (*pvert_list_)[u]; }
  inline WF_edge *GetEdge(int i) { return (i >= SizeOfEdgeList() || i < 0) ? NULL : (*pedge_list_)[i]; }
  inline WF_edge *GetNeighborEdge(int u) { return (u >= SizeOfVertList() || u < 0) ? NULL : (*pvert_list_)[u]->pedge_; }

  inline point GetPosition(int u) { assert(u < SizeOfVertList() && u >= 0); return((*pvert_list_)[u]->Position()); }
  inline int GetDegree(int u) { assert(u < SizeOfVertList() && u >= 0); return((*pvert_list_)[u]->Degree()); }

  inline int GetEndu(int i) { assert(i < SizeOfEdgeList() && i >= 0); return((*pedge_list_)[i]->ppair_->pvert_->ID()); }
  inline int GetEndv(int i) { assert(i < SizeOfEdgeList() && i >= 0); return((*pedge_list_)[i]->pvert_->ID()); }
  inline point GetCenterPos(int i) { assert(i < SizeOfEdgeList() && i >= 0); return((*pedge_list_)[i]->CenterPos()); }

  inline Vec3f GetCenterPos() const { return center_pos_; }
  inline Vec3f GetBaseCenterPos() const { return base_center_pos_; }

  inline bool isFixed(int u)	{ assert(u < SizeOfVertList() && u >= 0); return((*pvert_list_)[u]->isFixed()); }
  inline bool isPillar(int i)	{ assert(i < SizeOfEdgeList() && i >= 0); return((*pedge_list_)[i]->isPillar()); }

  inline double maxX() { return maxx_; }
  inline double minX() { return minx_; }
  inline double maxY() { return maxy_; }
  inline double minY() { return miny_; }
  inline double maxZ() { return maxz_; }
  inline double minZ() { return minz_; }

  inline double Norm(point u)
  {
	  return sqrt(u.x()*u.x() + u.y()*u.y() + u.z()*u.z());
  }

  inline double Dist(point u, point v)
  {
	  double dx = u.x() - v.x();
	  double dy = u.y() - v.y();
	  double dz = u.z() - v.z();
	  return sqrt(dx*dx + dy*dy + dz*dz);
  }

  inline point CrossProduct(point u, point v)
  {
	  return point(u.y() * v.z() - u.z() * v.y(), u.z() * v.x() - u.x() * v.z(),
				   u.x() * v.y() - u.y() * v.x());
  }

  inline double ArcHeight(point u, point v1, point v2)
  {
	  point alpha = u - v1;
	  point beta = v2 - v1;

	  return Norm(CrossProduct(alpha, beta)) / Norm(beta);
  }

 private:
  vector<WF_vert*> *pvert_list_;
  vector<WF_edge*> *pedge_list_;

  int	fixed_vert_;
  int	base_vert_;
  int	pillar_size_;
  int	ceiling_size_;
  int	layer_size_;

  double maxx_;
  double maxy_;
  double maxz_;
  double minx_;
  double miny_;
  double minz_;
  double basez_;

  Vec3f center_pos_;
  Vec3f base_center_pos_;
  float scaleV_;
  double unify_size_;
  double delta_tol_;
};