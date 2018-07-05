#include "choreo_task_sequence_planner/utils/DualGraph.h"

DualGraph::DualGraph()
{
  vert_list_ = NULL;
  edge_list_ = NULL;
  face_list_ = NULL;
}


DualGraph::DualGraph(WireFrame *ptr_frame)
{
  ptr_frame_ = ptr_frame;
  vert_list_ = NULL;
  edge_list_ = NULL;
  face_list_ = NULL;

  Init();
}


DualGraph::~DualGraph()
{
  Clear();
}


void DualGraph::Init()
{
  Clear();

  int N = ptr_frame_->SizeOfVertList();
  int M = ptr_frame_->SizeOfEdgeList();

  vert_list_ = new vector<DualVertex*>;
  vert_list_->resize(M);
  for (int i = 0; i < M; i++)
  {
    (*vert_list_)[i] = new DualVertex();
  }

  edge_list_ = new vector<DualEdge*>;

  face_list_ = new vector<DualFace*>;
  face_list_->resize(N);
  for (int i = 0; i < N; i++)
  {
    (*face_list_)[i] = new DualFace();
  }

  Nd_ = 0;
  Md_ = 0;
  Fd_ = 0;
  Fd_free_ = 0;

  exist_vert_.resize(N);
  fill(exist_vert_.begin(), exist_vert_.end(), 0);
  exist_edge_.resize(M);
  fill(exist_edge_.begin(), exist_edge_.end(), false);
}


void DualGraph::Clear()
{
  if (vert_list_ != NULL)
  {
    int N = vert_list_->size();
    for (int i = 0; i < N; i++)
    {
      delete (*vert_list_)[i];
      (*vert_list_)[i] = NULL;
    }
    delete vert_list_;
    vert_list_ = NULL;
  }

  if (edge_list_ != NULL)
  {
    int M = edge_list_->size();
    for (int i = 0; i < M; i++)
    {
      delete (*edge_list_)[i];
      (*edge_list_)[i] = NULL;
    }
    delete edge_list_;
    edge_list_ = NULL;
  }

  if (face_list_ != NULL)
  {
    int F = face_list_->size();
    for (int i = 0; i < F; i++)
    {
      delete (*face_list_)[i];
      (*face_list_)[i] = NULL;
    }
    delete face_list_;
    face_list_ = NULL;
  }
}


void DualGraph::Dualization()
{
  maxz_ = ptr_frame_->maxZ();
  minz_ = ptr_frame_->minZ();

  //maxz_ = -1e20;
  //minz_ = 1e20;

  //int M = ptr_frame_->SizeOfEdgeList();
  //for (int i = 0; i < M; i++)
  //{
  //	double z = ptr_frame_->GetCenterPos(i).z();
  //	if (z > maxz_)
  //	{
  //		maxz_ = z;
  //	}
  //	if (z < minz_)
  //	{
  //		minz_ = z;
  //	}
  //}

  // first time & all exsits
  fill(exist_vert_.begin(), exist_vert_.end(), 1);
  fill(exist_edge_.begin(), exist_edge_.end(), true);

  Establish();
}


void DualGraph::UpdateDualization(VectorXd *ptr_x)
{
  maxz_ = -1e20;

  int Nd = Nd_;

  fill(exist_vert_.begin(), exist_vert_.end(), 0);
  fill(exist_edge_.begin(), exist_edge_.end(), false);
  for (int e_id = 0; e_id < Nd; e_id++)
  {
    if ((*ptr_x)[e_id])
    {
      WF_edge *ei = ptr_frame_->GetEdge((*vert_list_)[e_id]->orig_id());
      WF_edge *ej = ei->ppair_;
      WF_vert *u = ej->pvert_;
      WF_vert *v = ei->pvert_;

      //if (ei->CenterPos().z() > maxz_)
      //{
      //	maxz_ = ei->CenterPos().z();
      //}

      if (u->Position().z() > maxz_)
      {
        maxz_ = u->Position().z();
      }
      if (v->Position().z() > maxz_)
      {
        maxz_ = v->Position().z();
      }

      exist_vert_[u->ID()]++;
      exist_vert_[v->ID()]++;
      exist_edge_[ei->ID()] = exist_edge_[ej->ID()] = true;
    }
  }

  // Release the last edge_list_
  int Md = edge_list_->size();
  for (int i = 0; i < Md; i++)
  {
    delete (*edge_list_)[i];
  }
  edge_list_->clear();

  Establish();
}


void DualGraph::Establish()
{
  int N = ptr_frame_->SizeOfVertList();
  int M = ptr_frame_->SizeOfEdgeList();

  Nd_ = 0;
  Fd_ = 0;
  Fd_free_ = 0;

  // vert list
  for (int i = 0; i < M; i++)
  {
    if (exist_edge_[i])
    {
      WF_edge *e = ptr_frame_->GetEdge(i);
      int j = e->ppair_->ID();
      if (i < j)
      {
        InsertVertex(e);
      }
    }
    else
    {
      (*vert_list_)[i]->SetDualId(-1);
    }
  }

  // face list
  for (int i = 0; i < N; i++)
  {
    if (exist_vert_[i] > 0)
    {
      InsertFace(ptr_frame_->GetVert(i));
    }
    else
    {
      (*face_list_)[i]->SetDualId(-1);
    }
  }

  // edge list
  for (int i = 0; i < N; i++)
  {
    if (exist_vert_[i] > 0)
    {
      if (ptr_frame_->GetDegree(i) > 1)
      {
        WF_vert *vert = ptr_frame_->GetVert(i);
        WF_edge *edge = ptr_frame_->GetNeighborEdge(i);
        double w = (ptr_frame_->GetPosition(i).z() - minz_) / (maxz_ - minz_);
        while (edge->pnext_ != NULL)
        {
          WF_edge *next_edge = edge->pnext_;
          //double w = ((edge->CenterPos().z() + next_edge->CenterPos().z()) / 2 - minz_)
          //	/ (maxz_ - minz_);
          InsertEdge(edge, next_edge, w, vert);
          edge = next_edge;
        }

        if (ptr_frame_->GetDegree(i) > 2)
        {
          WF_edge *next_edge = ptr_frame_->GetNeighborEdge(i);
          //double w = ((edge->CenterPos().z() + next_edge->CenterPos().z()) / 2 - minz_)
          //	/ (maxz_ - minz_);
          InsertEdge(edge, next_edge, w, vert);
        }
      }
    }
  }

  Md_ = edge_list_->size();

  //Debug();
}


int DualGraph::UpdateDualization(WF_edge *e)
{
  int i = e->ID();
  int j = e->ppair_->ID();
  int u = e->pvert_->ID();
  int v = e->ppair_->pvert_->ID();
  int dual_u = -1;
  int dual_v = -1;

  if (!exist_edge_[i])
  {
    if (exist_vert_[u] == 0)
    {
      dual_u = InsertFace(ptr_frame_->GetVert(u));
    }
    if (exist_vert_[v] == 0)
    {
      dual_v = InsertFace(ptr_frame_->GetVert(v));
    }
    exist_vert_[u]++;
    exist_vert_[v]++;

    InsertVertex(e);
    exist_edge_[i] = exist_edge_[j] = true;
  }

  // TODO: comment to enable seq print layer, edge in order might float
//  assert(dual_u == -1 || dual_v == -1);
  return (max(dual_u, dual_v));
}


int DualGraph::RemoveUpdation(WF_edge *e)
{
  int i = e->ID();
  int j = e->ppair_->ID();
  int	u = e->pvert_->ID();
  int	v = e->ppair_->pvert_->ID();
  int ret_dual = -1;

  if (exist_edge_[i])
  {
    exist_vert_[u]--;
    exist_vert_[v]--;
    if (exist_vert_[u] == 0)
    {
      ret_dual = DeleteFace(ptr_frame_->GetVert(u));
    }
    if (exist_vert_[v] == 0)
    {
      ret_dual = DeleteFace(ptr_frame_->GetVert(v));
    }

    DeleteVertex(e);
    exist_edge_[i] = exist_edge_[j] = false;
  }

  return ret_dual;
}


void DualGraph::InsertVertex(WF_edge *e)
{
  int i = e->ID();
  int j = e->ppair_->ID();
  (*vert_list_)[i]->SetDualId(Nd_);
  (*vert_list_)[j]->SetDualId(Nd_);
  (*vert_list_)[Nd_]->SetOrigId(i);
  (*vert_list_)[Nd_]->SetHeight((e->CenterPos()).z());
  Nd_++;
}


void DualGraph::InsertEdge(WF_edge *e1, WF_edge *e2, double w, WF_vert *vert)
{
  int u = (*vert_list_)[e1->ID()]->dual_id();
  int v = (*vert_list_)[e2->ID()]->dual_id();

  if (u != -1 && v != -1)
  {
    edge_list_->push_back(new DualEdge(u, v, w, vert));
  }
}


int DualGraph::InsertFace(WF_vert *p)
{
  int u = p->ID();
  int ret_dual = -1;
  if (p->isFixed())
  {
    (*face_list_)[u]->SetDualId(Fd_);
    (*face_list_)[Fd_]->SetOrigId(u);
  }
  else
  {
    if (Fd_free_ == Fd_)									// no fixed points
    {
      (*face_list_)[u]->SetDualId(Fd_);
      (*face_list_)[Fd_]->SetOrigId(u);
    }
    else
    {
      int dual_v = Fd_free_;								// first fixed point
      int orig_v = (*face_list_)[dual_v]->orig_id();

      (*face_list_)[u]->SetDualId(dual_v);
      (*face_list_)[dual_v]->SetOrigId(u);

      (*face_list_)[orig_v]->SetDualId(Fd_);
      (*face_list_)[Fd_]->SetOrigId(orig_v);
    }

    ret_dual = Fd_free_;
    Fd_free_++;
  }

  Fd_++;
  return ret_dual;
}


void DualGraph::DeleteVertex(WF_edge *e)
{
  int i = e->ID();
  int j = e->ppair_->ID();
  int ei = (*vert_list_)[i]->dual_id();
  int dual_e = Nd_ - 1;
  int orig_e1 = (*vert_list_)[dual_e]->orig_id();
  WF_edge *e_swap = ptr_frame_->GetEdge(orig_e1);
  int orig_e2 = e_swap->ppair_->ID();

  (*vert_list_)[i]->SetDualId(-1);
  (*vert_list_)[j]->SetDualId(-1);
  (*vert_list_)[orig_e1]->SetDualId(ei);
  (*vert_list_)[orig_e1]->SetDualId(ei);
  (*vert_list_)[ei]->SetOrigId(orig_e1);
  (*vert_list_)[ei]->SetHeight((e_swap->CenterPos()).z());

  Nd_--;
}


int DualGraph::DeleteFace(WF_vert *p)
{
  int orig_u = p->ID();
  int dual_u = (*face_list_)[orig_u]->dual_id();
  (*face_list_)[orig_u]->SetDualId(-1);
  (*face_list_)[dual_u]->SetOrigId(-1);

  for (int i = dual_u; i < Fd_ - 1; i++)
  {
    int j = i + 1;
    int orig_j = (*face_list_)[j]->orig_id();
    (*face_list_)[orig_j]->SetDualId(i);
    (*face_list_)[i]->SetOrigId(orig_j);
  }
  if (!p->isFixed())
  {
    Fd_free_--;
  }
  Fd_--;

  return dual_u;
}


void DualGraph::Debug()
{
  int M = ptr_frame_->SizeOfEdgeList();
  for (int i = 0; i < M; i++)
  {
    WF_edge *e = ptr_frame_->GetEdge(i);
    int end_id = e->pvert_->ID();
    int start_id = e->ppair_->pvert_->ID();
    printf("**********%d:\n", i);
    printf("start vertex: %d\n", start_id);
    printf("end vertex: %d\n", end_id);
    int dual_id = (*vert_list_)[i]->dual_id();
    printf("dual vertex: %d\n", dual_id);
    printf("original vertex: %d\n", (*vert_list_)[dual_id]->orig_id());
    getchar();
  }

  int N = ptr_frame_->SizeOfVertList();
  for (int i = 0; i < N; i++)
  {
    WF_vert *p = ptr_frame_->GetVert(i);
    printf("**********%d:\n", i);
    int dual_id = (*face_list_)[i]->dual_id();
    printf("dual vertex: %d\n", dual_id);
    printf("original vertex: %d\n", (*face_list_)[dual_id]->orig_id());
    getchar();
  }

  for (int i = 0; i < Md_; i++)
  {
    printf("%d %d\n", (*edge_list_)[i]->u(), (*edge_list_)[i]->v());
  }
}
