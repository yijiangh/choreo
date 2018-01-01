#include "framefab_task_sequence_planner/sequence_analyzers/SeqAnalyzer.h"

#include <moveit_msgs/CollisionObject.h>

namespace{
}// util namespace

SeqAnalyzer::SeqAnalyzer()
{
  ptr_frame_		= NULL;
  ptr_wholegraph_	= NULL;
  ptr_dualgraph_	= NULL;
  ptr_collision_	= NULL;
  ptr_path_		= NULL;

  Wp_ = 0;
  Wa_ = 0;
  Wi_ = 0;
  D_tol_ = 0;

  Nd_ = 0;

  terminal_output_ = false;
  file_output_ = false;
}

SeqAnalyzer::SeqAnalyzer(
    DualGraph			*ptr_dualgraph,
    QuadricCollision	*ptr_collision,
    Stiffness			*ptr_stiffness,
    FiberPrintPARM		*ptr_parm,
    char				*ptr_path,
    bool				terminal_output,
    bool				file_output
)
{
  ptr_frame_ = ptr_dualgraph->ptr_frame_;
  ptr_dualgraph_ = ptr_dualgraph;
  ptr_collision_ = ptr_collision;
  ptr_stiffness_ = ptr_stiffness;
  ptr_path_ = ptr_path;

  ptr_wholegraph_ = new DualGraph(ptr_frame_);

  Wp_ = ptr_parm->Wp_;
  Wa_ = ptr_parm->Wa_;
  Wi_ = ptr_parm->Wi_;
  D_tol_ = ptr_parm->seq_D_tol_;

  Nd_ = 0;

  terminal_output_ = terminal_output;
  file_output_ = file_output;
}

SeqAnalyzer::~SeqAnalyzer()
{
  delete ptr_wholegraph_;
  ptr_wholegraph_ = NULL;
}

bool SeqAnalyzer::SeqPrint()
{
  return true;
}

void SeqAnalyzer::PrintOutTimer()
{
}

void SeqAnalyzer::Init()
{
  ptr_wholegraph_->Dualization();
  Nd_ = ptr_wholegraph_->SizeOfVertList();

  D0_.resize(0);
  D0_.setZero();

  print_queue_.clear();

  angle_state_.clear();
  angle_state_.resize(Nd_);

  ptr_dualgraph_->Init();
}

void SeqAnalyzer::PrintPillars()
{
  int layer_size = ptr_frame_->SizeOfLayer();

  /* ranked by x */
  multimap<double, WF_edge*>base_queue;
  multimap<double, WF_edge*>::iterator it;
  for (int dual_i = 0; dual_i < Nd_; dual_i++)
  {
    WF_edge *e = ptr_frame_->GetEdge(ptr_wholegraph_->e_orig_id(dual_i));
    if (e->isPillar())
    {
      point center = e->CenterPos();
      base_queue.insert(make_pair(center.x(), e));
      UpdateStructure(e);
    }
  }

  for (it = base_queue.begin(); it != base_queue.end(); it++)
  {
    print_queue_.push_back(it->second);
  }

  if (terminal_output_)
  {
    fprintf(stderr, "Size of base queue: %d\n", base_queue.size());
  }

  /* angle state with pillars */
  for (int dual_i = 0; dual_i < Nd_; dual_i++)
  {
    WF_edge *e = ptr_frame_->GetEdge(ptr_wholegraph_->e_orig_id(dual_i));
    if (!ptr_dualgraph_->isExistingEdge(e))
    {
      ptr_collision_->DetectCollision(e, ptr_dualgraph_, angle_state_[dual_i]);
    }
  }
}

void SeqAnalyzer::UpdateStructure(WF_edge *e, bool update_collision)
{
//  if (terminal_output_)
//  {
//    upd_struct_.Start();
//  }

  int dual_upd = ptr_dualgraph_->UpdateDualization(e);

  /* modify D0 */
  if (dual_upd != -1)
  {
    int Ns = ptr_dualgraph_->SizeOfFreeFace();
    D0_.conservativeResize(6 * Ns);

    /* set initiate value by neighbors */
    int orig_u = ptr_dualgraph_->v_orig_id(dual_upd);
    WF_edge *eu = ptr_frame_->GetNeighborEdge(orig_u);

    VX sum_D(6);
    sum_D.setZero();
    int sum = 0;

    while (eu != NULL)
    {
      WF_vert *v = eu->pvert_;
      int dual_v = ptr_dualgraph_->v_dual_id(v->ID());
      if (dual_v != -1 && !v->isFixed())
      {
        VX tmp_D(6);
        for (int i = 0; i < 6; i++)
        {
          tmp_D[i] = D0_[6 * dual_v + i];
        }
        sum_D += tmp_D;
        sum++;
      }
      eu = eu->pnext_;
    }

    if (sum != 0)
    {
      sum_D /= sum;
    }
    for (int i = 0; i < 6; i++)
    {
      D0_[6 * (Ns - 1) + i] = sum_D[i];
    }
  }

//  if (terminal_output_)
//  {
//    upd_struct_.Stop();
//  }

  if(update_collision)
  {
    UpdateCollisionObjects(e);
  }
}

void SeqAnalyzer::RecoverStructure(WF_edge *e, bool recover_collision)
{
//  if (terminal_output_)
//  {
//    rec_struct_.Start();
//  }

  int dual_del = ptr_dualgraph_->RemoveUpdation(e);

  /* modify D0 */
  if (dual_del != -1)
  {
    int Ns = ptr_dualgraph_->SizeOfFreeFace();
    if (dual_del != Ns)
    {
      D0_.block(6 * dual_del, 0, 6 * (Ns - dual_del), 1) =
          D0_.block(6 * (dual_del + 1), 0, 6 * (Ns - dual_del), 1);
    }
    D0_.conservativeResize(6 * Ns);
  }

//  if (terminal_output_)
//  {
//    rec_struct_.Stop();
//  }

 if(recover_collision)
 {
    RecoverCollisionObjects(e);
 }
}

void SeqAnalyzer::UpdateStateMap(WF_edge *order_e, vector<vector<lld>> &state_map)
{
//  if (terminal_output_)
//  {
//    upd_map_.Start();
//  }

  int dual_i = ptr_wholegraph_->e_dual_id(order_e->ID());
  int Nd = ptr_wholegraph_->SizeOfVertList();
  for (int dual_j = 0; dual_j < Nd; dual_j++)
  {
    WF_edge * target_e = ptr_frame_->GetEdge(ptr_wholegraph_->e_orig_id(dual_j));
    if (dual_i != dual_j && !ptr_dualgraph_->isExistingEdge(target_e))
    {
//      if (terminal_output_)
//      {
//        upd_map_collision_.Start();
//      }

      vector<lld> tmp(3);
      ptr_collision_->DetectCollision(target_e, order_e, tmp);

//      if (terminal_output_)
//      {
//        upd_map_collision_.Stop();
//      }

      for (int k = 0; k < 3; k++)
      {
        state_map[k].push_back(angle_state_[dual_j][k]);
      }
      ptr_collision_->ModifyAngle(angle_state_[dual_j], tmp);
    }
  }

//  if (terminal_output_)
//  {
//    upd_map_.Stop();
//  }
}


void SeqAnalyzer::RecoverStateMap(WF_edge *order_e, vector<vector<lld>> &state_map)
{
//  if (terminal_output_)
//  {
//    rec_map_.Start();
//  }

  int dual_i = ptr_wholegraph_->e_dual_id(order_e->ID());
  int Nd = ptr_wholegraph_->SizeOfVertList();
  int p = 0;
  for (int dual_j = 0; dual_j < Nd; dual_j++)
  {
    WF_edge * target_e = ptr_frame_->GetEdge(ptr_wholegraph_->e_orig_id(dual_j));
    if (dual_i != dual_j && !ptr_dualgraph_->isExistingEdge(target_e))
    {
      for (int k = 0; k < 3; k++)
      {
        angle_state_[dual_j][k] = state_map[k][p];
      }
      p++;
    }
  }

//  if (terminal_output_)
//  {
//    rec_map_.Stop();
//  }
}

void SeqAnalyzer::UpdateCollisionObjects(WF_edge* e)
{
  // for connected vert of this edge

  // shrink the connected sides of this edge

  // add this edge to the planning scene

  // query connected & existing edges for these nodes, shrink them

  // pop these edges from existing planning_scene, replace with the shrinked ones

}

void SeqAnalyzer::RecoverCollisionObjects(WF_edge* e)
{
  // pop this edge from planning scene

  // query connected & existing edges for the connected node, shrink them

  // pop these edges from existing planning_scene, replace with the shrinked ones

}

bool SeqAnalyzer::TestifyStiffness(WF_edge *e)
{
//  if (terminal_output_)
//  {
//    test_stiff_.Start();
//  }

  /* insert a trail edge */
  UpdateStructure(e);

  /* examinate stiffness on printing subgraph */
  ptr_stiffness_->Init();

  int Ns = ptr_dualgraph_->SizeOfFreeFace();
  VX D(Ns * 6);
  D.setZero();

  bool bSuccess = ptr_stiffness_->CalculateD(D);

  if (bSuccess)
  {
    for (int k = 0; k < Ns; k++)
    {
      VX offset(3);
      for (int t = 0; t < 3; t++)
      {
        offset[t] = D[k * 6 + t];
      }

      if (offset.norm() >= D_tol_)
      {
        //printf("$$$Stiffness offset: %lf\n", offset.norm());
        bSuccess = false;
        break;
      }
    }
  }

  D0_ = D;

  /* remove the trail edge */
  RecoverStructure(e);

//  if (terminal_output_)
//  {
//    test_stiff_.Stop();
//  }

  return bSuccess;
}

bool SeqAnalyzer::TestRobotKinematics(WF_edge *e)
{

  return false;
}

bool SeqAnalyzer::InputPrintOrder(vector<int> &print_queue)
{
  print_queue_.clear();

  int Nq = print_queue.size();
  for (int i = 0; i < Nq; i++)
  {
    WF_edge *e = ptr_frame_->GetEdge(print_queue[i]);
    if (e == NULL)
    {
      print_queue_.clear();
      return false;
    }
    print_queue_.push_back(e);
  }

  return true;
}


void SeqAnalyzer::OutputPrintOrder(vector<WF_edge*> &print_queue)
{
  print_queue.clear();

  int Nq = print_queue_.size();
  for (int i = 0; i < Nq; i++)
  {
    print_queue.push_back(print_queue_[i]);
  }
}
