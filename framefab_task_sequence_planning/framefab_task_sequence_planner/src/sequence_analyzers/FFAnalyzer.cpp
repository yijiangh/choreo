#include "framefab_task_sequence_planner/sequence_analyzers/FFAnalyzer.h"

FFAnalyzer::~FFAnalyzer()
{
}

bool FFAnalyzer::SeqPrint()
{
  FF_analyzer_.Start();

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

  int print_until = layer_size;

  for (int l = 0; l < print_until; l++)
  {
    fprintf(stderr, "Size of layer %d is %d\n", l, (int)layers_[l].size());
  }

  /* print starting from the first layer */
  bool bSuccess = true;
  for (int l = 0; l < print_until; l++)
  {
    /*
    * Nl: number of dual verts in current layer
    * h : head for printing queue of the layer
    * t : tail for printing queue of the layer
    */

    int Nl = layers_[l].size();
    int h = print_queue_.size(); // print queue size so far
    int t;

    if (l == 0)
    {
      /* set pillars as starting edges */
      PrintPillars();
      continue;
    }
    else
    {
      t = h + Nl;
    }

    if (h == t)
    {
      continue;
    }

    /* max_z_ and min_z_ in current layer */
    min_base_dist_ = min_z_ = 1e20;
    max_base_dist_ = max_z_ = -1e20;

    for (int i = 0; i < Nl; i++)
    {
      WF_edge* e = layers_[l][i];
      point u = e->pvert_->Position();
      point v = e->ppair_->pvert_->Position();
      min_z_ = min(min_z_, (double)min(u.z(), v.z()));
      max_z_ = max(max_z_, (double)max(u.z(), v.z()));

      const auto& st_pt_msg = frame_msgs_[e->ID()].start_pt;
      const auto& end_pt_msg = frame_msgs_[e->ID()].end_pt;

      // distance to robot base (world_frame 0,0,0)
      double dist = sqrt(pow((st_pt_msg.x + end_pt_msg.x)/2, 2)
                             + pow((st_pt_msg.y + end_pt_msg.y)/2, 2)
                             + pow((st_pt_msg.z + end_pt_msg.z)/2, 2));


      min_base_dist_ = min(min_base_dist_, dist);
      max_base_dist_ = max(max_base_dist_, dist);
    }

    if (!GenerateSeq(l, h, t))
    {
      fprintf(stderr,
              "All possible start edge at layer %d has been tried but no feasible sequence is obtained.\n", l);
      bSuccess = false;
      break;
    }
  }

  fprintf(stderr, "\n\nFFAnalyzer seq search finished:\n Number of edges: %d\nNumber of partial assignment visited: %d, Number of backtrack: %d\n\n",
          ptr_wholegraph_->SizeOfVertList(), num_p_assign_visited_, num_backtrack_);

  FF_analyzer_.Stop();
  return bSuccess;
}

bool FFAnalyzer::SeqPrintLayer(int layer_id)
{
  assert(ptr_frame_->SizeOfLayer() > layer_id);

  FF_analyzer_.Start();

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

  fprintf(stderr, "Size of target layer %d is %d\n", layer_id, (int)layers_[layer_id].size());

  bool bSuccess = true;

  for(int l=0; l < layer_id; l++)
  {
    std::cout << "layer #" << l << std::endl;

    for(int s=0; s < layers_[l].size(); s++)
    {
      WF_edge* e = layers_[l][s];

      print_queue_.push_back(e);

      // update printed graph
      UpdateStructure(e, update_collision_);

      if(layer_id - 1 == l)
      {
        // only update state map for edges in target layer
        int Nd = layers_[layer_id].size();

        for (int k = 0; k < Nd; k++)
        {
          WF_edge *ek = layers_[layer_id][k];
          int dual_k = ptr_wholegraph_->e_dual_id(ek->ID());

          std::vector <lld> tmp(3);
          ptr_collision_->DetectCollision(ek, e, tmp);

          ptr_collision_->ModifyAngle(angle_state_[dual_k], tmp);
        }
      }
    }
  }

  std::cout << "Update finished." << std::endl;

  int target_size = 0;

  for(int lb = layer_id; lb < layer_id+1; lb++)
  {
    // search in target layer
    int Nl = layers_[lb].size();
    int h = print_queue_.size(); // print queue size so far
    int t = h + Nl;

    target_size += Nl;

    /* max_z_ and min_z_ in current layer */
    min_base_dist_ = min_z_ = 1e20;
    max_base_dist_ = max_z_ = -1e20;

    for (int i = 0; i < Nl; i++)
    {
      WF_edge *e = layers_[lb][i];
      point u = e->pvert_->Position();
      point v = e->ppair_->pvert_->Position();
      min_z_ = min(min_z_, (double) min(u.z(), v.z()));
      max_z_ = max(max_z_, (double) max(u.z(), v.z()));

      const auto& st_pt_msg = frame_msgs_[e->ID()].start_pt;
      const auto& end_pt_msg = frame_msgs_[e->ID()].end_pt;

      // distance to robot base (world_frame 0,0,0)
      double dist = sqrt(pow((st_pt_msg.x + end_pt_msg.x)/2, 2)
                             + pow((st_pt_msg.y + end_pt_msg.y)/2, 2)
                             + pow((st_pt_msg.z + end_pt_msg.z)/2, 2));

      min_base_dist_ = min(min_base_dist_, dist);
      max_base_dist_ = max(max_base_dist_, dist);
    }

    if (!GenerateSeq(lb, h, t))
    {
      fprintf(stderr,
              "All possible start edge at layer %d has been tried but no feasible sequence is obtained.\n", lb);
      bSuccess = false;
      break;
    }
  }

  fprintf(stderr, "****Reminder: Start of target layers is %d\n", Nd_ - target_size);

  FF_analyzer_.Stop();
  return bSuccess;
}

bool FFAnalyzer::GenerateSeq(int l, int h, int t)
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

    if (cost == -2)
    {
      return false;
    }

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
    std::vector<std::vector<lld>> tmp_angle(3);
    UpdateStateMap(ej, tmp_angle);

    num_p_assign_visited_++;

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

    num_backtrack_++;

    if (terminal_output_)
    {
      fprintf(stderr, "---- Backtrack - head %d/tail %d\n", h, t);
    }

    cnt++;
  }

  return false;
}


double FFAnalyzer::GenerateCost(WF_edge *ei, WF_edge *ej, const int h, const int t, const int l)
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
    double z = 0.0;

    // use dist to robot base as heuristic
    const auto& st_pt_msg = frame_msgs_[ej->ID()].start_pt;
    const auto& end_pt_msg = frame_msgs_[ej->ID()].end_pt;

    // distance to robot base (world_frame 0,0,0)
    double dist = sqrt(pow((st_pt_msg.x + end_pt_msg.x)/2, 2)
                           + pow((st_pt_msg.y + end_pt_msg.y)/2, 2)
                           + pow((st_pt_msg.z + end_pt_msg.z)/2, 2));

//    if(min_base_dist_ != max_base_dist_)
//    {
//      P = 1 - (dist - min_base_dist_) / (max_base_dist_ - min_base_dist_);
//    }

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
        fprintf(stderr, "...collision test failed at edge #%d.\n", ej->ID() / 2);
      }

      fprintf(stderr, "fail edge vert u: (%f, %f, %f), vert v: (%f, %f, %f)\n\n",
              ej->pvert_->Position().x(), ej->pvert_->Position().y(), ej->pvert_->Position().z(),
              ej->ppair_->pvert_->Position().x(), ej->ppair_->pvert_->Position().y(), ej->ppair_->pvert_->Position().z());

      return -2;
    }
    else
    {
      /* robot kinematics test */
      if(update_collision_)
      {
        if (!TestRobotKinematics(ej, angle_state_[dual_j]))
        {
          /* examination failed */
          if (terminal_output_)
          {
            fprintf(stderr, "...robot kinematics test failed at edge #%d.\n", ej->ID() / 2);
            fprintf(stderr, "fail edge vert u: (%f, %f, %f), vert v: (%f, %f, %f)\n\n",
                    ej->pvert_->Position().x(), ej->pvert_->Position().y(), ej->pvert_->Position().z(),
                    ej->ppair_->pvert_->Position().x(), ej->ppair_->pvert_->Position().y(), ej->ppair_->pvert_->Position().z());
          }

          return -2;
        }
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

  // ej exists already, skip
  return -1;
}


void FFAnalyzer::PrintOutTimer()
{
  printf("***FFAnalyzer timer result:\n");
  FF_analyzer_.Print("FFAnalyzer:");

  if (terminal_output_)
  {
    upd_struct_.Print("UpdateStructure:");
    rec_struct_.Print("RecoverStructure:");
    upd_map_.Print("UpdateStateMap:");
    upd_map_collision_.Print("DetectCollision:");
    rec_map_.Print("RecoverStateMap:");
    test_stiff_.Print("TestifyStiffness:");
  }
  else
  {
    printf("***FFAnalyzer detailed timing turned off.\n");
  }
}
