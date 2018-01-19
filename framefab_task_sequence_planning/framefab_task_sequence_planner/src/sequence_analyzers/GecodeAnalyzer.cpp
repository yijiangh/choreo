//
// Created by yijiangh on 1/17/18.
//

#include "framefab_task_sequence_planner/sequence_analyzers/GecodeAnalyzer.h"

#include <gecode/gist.hh>

GecodeAnalyzer::~GecodeAnalyzer()
{
}

bool GecodeAnalyzer::SeqPrint()
{
  gecode_analyzer_.Start();

  Init();

  /* split layers */
  /* label stores layer index of each dual node */
  int layer_size = ptr_frame_->SizeOfLayer();
  layers_.clear();
  layers_.resize(layer_size);
  for (int i = 0; i < Nd_; i++)
  {
    WF_edge *e = ptr_frame_->GetEdge(ptr_wholegraph_->e_orig_id(i));
    layers_[e->Layer()].push_back(e);
  }

  /* printing */
  /* set pillars as starting edges */
  PrintPillars();

  for (int l = 0; l < layer_size; l++)
  {
    fprintf(stderr, "Size of layer %d is %d\n", l, (int)layers_[l].size());
  }

  /* print starting from the first layer */
  bool bSuccess = true;
  for (int l = 0; l < layer_size; l++)
  {
    int Nl = layers_[l].size();

    /* max_z_ and min_z_ in current layer */
    min_z_ = 1e20;
    max_z_ = -min_z_;

    for (int i = 0; i < Nl; i++)
    {
      WF_edge* e = layers_[l][i];
      point u = e->pvert_->Position();
      point v = e->ppair_->pvert_->Position();
      min_z_ = min(min_z_, (double)min(u.z(), v.z()));
      max_z_ = max(max_z_, (double)max(u.z(), v.z()));
    }

    // compute input for gecode solver
    int n = layers_[l].size();
    int m = ptr_collision_->Divide();
    std::vector<int> A, G, T;

    ComputeGecodeInput(layers_[l], A, G, T);

    // feed data in gecode option
    const Gecode::AssemblySequenceOptions opt("Gecode-TS-Planning", A, G, T, n, m);

    Gecode::AssemblySequence* ptr_as = new Gecode::AssemblySequence(opt);
    Gecode::DFS<Gecode::AssemblySequence> e(ptr_as, opt);

    delete(ptr_as);

    while (Gecode::AssemblySequence* s = e.next())
    {
      s->print();
      delete s;
    }

    // gecode solve
//    Gecode::Script::run<Gecode::AssemblySequence, Gecode::DFS, Gecode::AssemblySequenceOptions>(opt);

    // how to get result???
    // push in print queue
    // update dual graph
  }

  gecode_analyzer_.Stop();
  return bSuccess;
}

void GecodeAnalyzer::ComputeGecodeInput(const std::vector<WF_edge*>& layer_e,
                                        std::vector<int>& A, std::vector<int>& G, std::vector<int>& T)
{
  int n = layer_e.size();
  int m = ptr_collision_->Divide();

  A = std::vector<int>(n*n, 0);
  G = std::vector<int>(n, 0);
  T = std::vector<int>(n*n*m, 0);

  for(int i=0; i < n; i++)
  {
    WF_edge* e = layer_e[i];

    // connectivity
    // if node exist, grounded
    if(ptr_dualgraph_->isExistingEdge(e) && e->isPillar())
    {
      G[i] = 1;
    }

    // for current layer's edge, if share node = 1
    std::vector<int> connect_vert_id(2);
    connect_vert_id[0] = ptr_frame_->GetEndu(e->ID());
    connect_vert_id[1] = ptr_frame_->GetEndv(e->ID());

    for(const auto id : connect_vert_id)
    {
      WF_edge* eu = ptr_frame_->GetNeighborEdge(id);

      while (eu != NULL)
      {
        if (eu->ID() == e->ID() || eu->ID() == e->ppair_->ID())
        {
          eu = eu->pnext_;
          continue;
        }

        for(int j=0; j < n; j++)
        {
          if(eu->ID() == layer_e[j]->ID() || eu->ID() == layer_e[j]->ppair_->ID())
          {
            A[i*n + j] = 1;
          }
        }

        eu = eu->pnext_;
      }
    }

    int dual_i = ptr_wholegraph_->e_dual_id(e->ID());

    for (int j = 0; j < n; j++)
    {
      WF_edge* ej = layer_e[j];
      int dual_j = ptr_wholegraph_->e_dual_id(ej->ID());

      if (dual_i != dual_j)
      {
        std::vector<lld> tmp(3);
        ptr_collision_->DetectCollision(ej, e, tmp);

        std::vector<int> tmp_vector = ptr_collision_->ConvertCollisionMapToIntMap(tmp);

        for(int k=0; k<m; k++)
        {
          T[(i*n + j)*m + k] = tmp_vector[k];
        }
      }
    }

  } // end loop for i
}

bool GecodeAnalyzer::GenerateSeq(int l, int h, int t)
{
  /* last edge */
  assert(h != 0); // there must be pillars
  WF_edge *ei = print_queue_[h - 1];

  if (terminal_output_)
  {
    fprintf(stderr, "-----------------------------------\n");
    fprintf(stderr, "Searching edge #%d in layer %d, head %d, tail %d\n",
            ei->ID() / 2, l, h, t);
  }

  /* exit */
  if (h == t)
  {
    if (terminal_output_)
    {
      fprintf(stderr, "++++ Head meets tail! Current layer solution found.\n\n");
    }

    return true;
  }

  /* next choice */
  multimap<double, WF_edge*> choice;
  multimap<double, WF_edge*>::iterator it;

  /* next edge in current layer */
  int Nl = layers_[l].size();

  // for all the edges in current layer, generate cost (pruning in cost generation)
  // constraint propagation (arc-consistency) in current layer
  for (int j = 0; j < Nl; j++)
  {
    WF_edge *ej = layers_[l][j];

    /* cost weight */
    double cost = GenerateCost(ei, ej, h, t, l);
    if (cost != -1)
    {
      // eligible candidate, cost = -1 if stiffness or kinematic check not pass
      choice.insert(pair<double, WF_edge*>(cost, ej));
    }
  }

  int cnt = 0;
  /* ranked by weight */
  for (it = choice.begin(); it != choice.end(); it++)
  {
    WF_edge *ej = it->second;
    print_queue_.push_back(ej);

    /* update printed subgraph */
    UpdateStructure(ej, update_collision_);

    // update collision (geometric domain)
    // tmp is the pruned domain by direct arc consistency pruning
    vector <vector<lld>> tmp_angle(3);
    UpdateStateMap(ej, tmp_angle);

    if (terminal_output_)
    {
      fprintf(stderr, "Choose edge #%d in with cost %lf - candidates %d/%d \n\n",
              ej->ID() / 2, it->first, cnt, (int)choice.size());
    }

    if (GenerateSeq(l, h + 1, t))
    {
      return true;
    }

    // backtrack
    RecoverStateMap(ej, tmp_angle);
    RecoverStructure(ej, update_collision_);

    print_queue_.pop_back();

    if (terminal_output_)
    {
      fprintf(stderr, "---- Backtrack - head %d/tail %d\n", h, t);
    }

    cnt++;
  }

  return false;
}


double GecodeAnalyzer::GenerateCost(WF_edge *ei, WF_edge *ej, const int h, const int t, const int l)
{
  int orig_j = ej->ID();
  int dual_j = ptr_wholegraph_->e_dual_id(orig_j);

  // check arc consistency for all unprinted elements
  if (!ptr_dualgraph_->isExistingEdge(ej))
  {
    double	P = 0; // stabiliy weight
    double  A = 0; // adjacency weight
    double	I = 0; // influence weight

    if (terminal_output_)
    {
      fprintf(stderr, "Look-ahead in layer %d/%d - head %d/tail %d: Attempting edge #%d, queue size %d\n",
              l, (int)layers_.size(), h, t, orig_j / 2, (int)print_queue_.size());
    }

    /* stabiliy weight */
    int uj = ptr_frame_->GetEndu(orig_j);
    int vj = ptr_frame_->GetEndv(orig_j);
    bool exist_uj = ptr_dualgraph_->isExistingVert(uj);
    bool exist_vj = ptr_dualgraph_->isExistingVert(vj);
    double z;

    if(max_z_ != min_z_)
    {
      z = (ej->CenterPos().z() - min_z_) / (max_z_ - min_z_);
    }
    else
    {
      z = 0.0;
    }

    // prune floating edge
    if (exist_uj && exist_vj)
    {
      /* edge j share two ends with printed structure */
      P = z;
    }
    else
    {
      if (exist_uj || exist_vj)
      {
        /* edge j share one end with printed structure */
        double ang;
        point pos_uj = ptr_frame_->GetPosition(uj);
        point pos_vj = ptr_frame_->GetPosition(vj);

        if (exist_uj)
        {
          ang = Geometry::angle(point(0, 0, 1), pos_vj - pos_uj);
        }
        else
        {
          ang = Geometry::angle(point(0, 0, 1), pos_uj - pos_vj);
        }

        P = z * exp(ang);
      }
      else
      {
        if (terminal_output_)
        {
          fprintf(stderr, "...floating edge, skip\n\n");
        }
        return -1;
      }
    }

    /* collision test */
    int free_angle = ptr_collision_->ColFreeAngle(angle_state_[dual_j]);
    if (free_angle == 0)
    {
      if (terminal_output_)
      {
        fprintf(stderr, "...collision test failed at edge #%d.\n\n", ej->ID() / 2);
      }
      return -1;
    }
    else
    {
//      if(free_angle < ptr_collision_->Divide() * 0.15)
//      ROS_INFO_STREAM("robot kinematics check: free angle num: " << free_angle << "/"
//                                                                 << int(ptr_collision_->Divide()));
      /* robot kinematics test */
      if(update_collision_)
      {
        if (!TestRobotKinematics(ej, angle_state_[dual_j]))
        {
          /* examination failed */
          if (terminal_output_)
          {
            fprintf(stderr, "...robot kinematics test failed at edge #%d.\n\n", ej->ID() / 2);
          }
          return -1;
        }
//        else
//        {
//          ROS_INFO_STREAM("Robot kinematics test passed.");
//        }
      }
    }

    /* stiffness test */
    if (!TestifyStiffness(ej))
    {
      /* examination failed */
      if (terminal_output_)
      {
        fprintf(stderr, "...stiffness examination failed at edge #%d.\n\n", ej->ID() / 2);
      }
      return -1;
    }

    /* adjacency weight */
//    if (ei == NULL)
//    {
//      A = 0;
//    }
//    else
//    {
//      int ui = ei->pvert_->ID();
//      int vi = ei->ppair_->pvert_->ID();
//      if (ui == uj || ui == vj || vi == uj || vi == vj)
//      {
//        A = 0;
//      }
//      else
//      {
//        A = 1.0;
//      }
//    }

    // Forward Checking
    // limit forward checking only to current layer
    /* influence weight */
    int Nl = layers_[l].size();
    int remaining_num = 0;
    double forward_checking_factor = 1.0;

    for (int k = 0; k < Nl; k++)
    {
      WF_edge* ek = layers_[l][k];
      int dual_k = ptr_wholegraph_->e_dual_id(ek->ID());

      // considered edge is not ej itself and it's not printed yet
      if (dual_j != dual_k && !ptr_dualgraph_->isExistingEdge(ek))
      {
        vector<lld> tmp(3);
        ptr_collision_->DetectCollision(ek, ej, tmp);

        for (int o = 0; o < 3; o++)
        {
          tmp[o] |= angle_state_[dual_k][o];
        }

        int future_angle = ptr_collision_->ColFreeAngle(tmp);

        if(0 == future_angle)
        {
          fprintf(stderr, "...FC pruning - collision test failed at edge #%d to future edge #%d.\n\n",
                  ej->ID() / 2, ek->ID() / 2);
          return -1;
        }

        double d_ratio = (double)future_angle / (double)ptr_collision_->Divide();
        I += exp(- forward_checking_factor * d_ratio * d_ratio);

        remaining_num++;
      }
    }

    if(0 != remaining_num)
    {
      I /= remaining_num;
    }
    else
    {
      I = 0.0;
    }

    double cost = Wp_*P + Wa_*A + Wi_*I;

    if (terminal_output_)
    {
      fprintf(stderr, "P: %lf, A: %lf, I: %lf\ncost: %f\n\n", P, A, I, cost);
    }
    return cost;
  } // end if exist_edge ej

  return -1;
}


void GecodeAnalyzer::PrintOutTimer()
{
  printf("***GecodeAnalyzer timer result:\n");
  gecode_analyzer_.Print("GecodeAnalyzer:");
}

void GecodeAnalyzer::debug()
{
  using namespace Gecode;

//  Gist::Print<SendMostMoney> p("Print solution");
//  Gist::Options o;
//  o.inspect.click(&p);
//
//  Gecode::Gist::dfs(&gecode_instance_, o);

  // Branch & Bound search engine
//  Gecode::BAB<Gecode::SendMostMoney> e(&gecode_instance_);
//
//  while(Gecode::SendMostMoney* s = e.next())
//  {
//    s->print();
//    delete s;
//  }
}