/*
* ==========================================================================
*		This file is part of the implementation of
*
*		<FrameFab: Robotic Fabrication of Frame Shapes>
*		Yijiang Huang, Juyong Zhang, Xin Hu, Guoxian Song, Zhongyuan Liu, Lei Yu, Ligang Liu
*		In ACM Transactions on Graphics (Proc. SIGGRAPH Asia 2016)
----------------------------------------------------------------------------
*		class:	DualGraph
*
*		Description:	dual graph is the basic data structure used to perform ADMMCut Algorthim
*
*		Version:  2.0
*		Created:  Oct/10/2015
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
#include <iostream>
#include <Eigen/Core>

#include "framefab_task_sequence_planner/utils/WireFrame.h"

using namespace std;
using namespace Eigen;


class DualVertex
{
public:
	DualVertex(){ orig_id_ = -1; dual_id_ = -1; }
	~DualVertex(){}

public:
	void	SetOrigId(int orig_id)	{ orig_id_ = orig_id; }
	void	SetDualId(int dual_id)	{ dual_id_ = dual_id; }
	void	SetHeight(double height){ height_ = height; }

	int		orig_id()	const { return orig_id_; }
	int		dual_id()	const { return dual_id_; }
	double	Height()	const { return height_; }

private:
	int		orig_id_;										// indexed by dual edge id
	int		dual_id_;										// indexed by original edge id
	double	height_;										// z of central points on this original edge
															// indexed by dual id
};


class DualEdge
{
public:
	DualEdge(){}
	DualEdge(int u, int v, double w, WF_vert* vert)
	{
		u_ = u;
		v_ = v;
		w_ = w;
		pvert_ = vert;
	}
	~DualEdge(){}

public:
	int			u()				const { return u_; }
	int			v()				const { return v_; }
	double		w()				const { return w_; }
	WF_vert*	CentralVert()	const { return pvert_; }

private:
	int			u_;
	int			v_;
	double		w_;
	WF_vert*	pvert_;
};


class DualFace
{
public:
	DualFace(){ orig_id_ = -1; dual_id_ = -1; }
	~DualFace(){}

public:
	void	SetOrigId(int orig_id)	{ orig_id_ = orig_id; }
	void	SetDualId(int dual_id)	{ dual_id_ = dual_id; }

	int		orig_id()				const { return orig_id_; }
	int		dual_id()				const { return dual_id_; }

private:
	int		orig_id_;										// indexed by dual vertex id
	int		dual_id_;										// indexed by original vertex id
};


class DualGraph
{
public:
	DualGraph();
	DualGraph(WireFrame *ptr_frame);
	~DualGraph();

public:
	void	Init();
	void	Clear();

	void	Dualization();									// dualization on the whole frame

	void	UpdateDualization(VectorXd *ptr_x);				// update from graphcut
	void	Establish();

	int		UpdateDualization(WF_edge *e);					// insert a trail edge ei from frame 
	int		RemoveUpdation(WF_edge *e);						// remove the trail edge

	void	InsertVertex(WF_edge *e);
	void	InsertEdge(WF_edge *e1, WF_edge *e2, double w, WF_vert *vert);
	int		InsertFace(WF_vert *p);							// insert a dual face at the end of free face
	void	DeleteVertex(WF_edge *e);
	int		DeleteFace(WF_vert *p);							// delete a dual face by moving

	vector<DualVertex*>		*GetVertList()		{ return vert_list_; }
	vector<DualEdge*>		*GetEdgeList()		{ return edge_list_; }
	vector<DualFace*>		*GetFaceList()		{ return face_list_; }

	int		SizeOfVertList()	{ return Nd_; }
	int		SizeOfEdgeList()	{ return Md_; }
	int		SizeOfFaceList()	{ return Fd_; }
    int     SizeOfFreeFace()    { return Fd_free_; }

	int		u(int ei)			{ return (*edge_list_)[ei]->u(); }
	int		v(int ei)			{ return (*edge_list_)[ei]->v(); }
	int		e_orig_id(int u)	{ return (*vert_list_)[u]->orig_id(); }
	int		e_dual_id(int u)	{ return (*vert_list_)[u]->dual_id(); }
	int		v_orig_id(int i)	{ return (*face_list_)[i]->orig_id(); }
	int		v_dual_id(int i)	{ return (*face_list_)[i]->dual_id(); }

	double	Weight(int ei)		{ return (*edge_list_)[ei]->w(); }
	WF_vert*CentralVert(int ei)	{ return (*edge_list_)[ei]->CentralVert(); }
	double	Height(int ei)		{ return (*vert_list_)[ei]->Height(); }
	double	maxZ()				{ return maxz_; }
	double	minZ()				{ return minz_; }

	bool isExistingVert(int u)		{ return (exist_vert_[u] > 0); }
	bool isExistingEdge(WF_edge *e)	{ return exist_edge_[e->ID()]; }

	bool isAdjacent(int i, int j)
	{
		WF_edge *e1 = ptr_frame_->GetEdge(e_orig_id(i));
		WF_edge *e2 = ptr_frame_->GetEdge(e_orig_id(j));
		int u1 = e1->ppair_->pvert_->ID();
		int v1 = e1->pvert_->ID();
		int u2 = e2->ppair_->pvert_->ID();
		int v2 = e2->pvert_->ID();

		if (u1 == u2 || u1 == v2 || v1 == u2 || v1 == v2)
		{
			return true;
		}
		else
		{
			return false;
		}
	}

	void	Debug();

public:
	WireFrame				*ptr_frame_;

private:
	vector<DualEdge*>		*edge_list_;				// dual edge: original edge -> original edge
	vector<DualVertex*>		*vert_list_;				// dual vert: original edge
	vector<DualFace*>		*face_list_;				// dual face: original vert

	vector<int>				exist_vert_;				// indexed by original id
	vector<bool>			exist_edge_;				// indexed by original id

	vector<vector<bool>>	is_adjacent_;

	int						Nd_;
	int						Md_;
	int						Fd_;
    int                     Fd_free_;

	double					maxz_;
	double					minz_;
};
