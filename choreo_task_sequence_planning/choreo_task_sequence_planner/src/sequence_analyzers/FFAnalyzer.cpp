#include <numeric>

#include "choreo_task_sequence_planner/utils/GCommon.h"
#include "choreo_task_sequence_planner/sequence_analyzers/FFAnalyzer.h"

FFAnalyzer::~FFAnalyzer()
{
}

bool FFAnalyzer::SeqPrint()
{
  FF_analyzer_.Start();

  Init();

  ROS_INFO_STREAM("[TSP] FF Analyzer init.");

  /* split layers */
  /* label stores layer index of each dual node */
  int layer_size = ptr_frame_->SizeOfLayer();
  layers_.clear();
  layers_.resize(layer_size);
  for (int i = 0; i < Nd_; i++)
  {
    WF_edge *e = ptr_frame_->GetEdge(ptr_wholegraph_->e_orig_id(i));
    assert(e);
    layers_[e->Layer()].push_back(e);
  }

  int print_until = layer_size;
//  int print_until = 17;

  assert(print_until <= layer_size);

  for (int l = 0; l < print_until; l++)
  {
//    if(terminal_output_)
//    {
    fprintf(stderr, "Size of layer %d is %d\n", l, (int) layers_[l].size());
//    }
    assert((int)layers_[l].size() != 0);
  }

  /* print starting from the first layer */
  bool bSuccess;
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

    t = h + Nl;

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

    search_rerun_ = 0;
    bSuccess = false;

    while(!bSuccess && search_rerun_ < 1)
    {
      bSuccess = GenerateSeq(l, h, t);
      search_rerun_++;
    }

    if(!bSuccess)
    {
      ROS_ERROR(
          "All possible start edge at layer %d has been tried but no feasible sequence is obtained after %d iterations.", l, search_rerun_);
      break;
    }
  }

  fprintf(stderr,
          "\n\nFFAnalyzer seq search finished:\n Number of edges: %d\nNumber of partial assignment visited: %d, Number of backtrack: %d\n\n",
          ptr_wholegraph_->SizeOfVertList(), num_p_assign_visited_, num_backtrack_);

  FF_analyzer_.Stop();
  return bSuccess;
}

bool FFAnalyzer::GenerateSeq(int l, int h, int t)
{
  /* last edge */
  if(0 != h)
  {
    WF_edge *ei = print_queue_[h - 1];

    if (terminal_output_)
    {
      fprintf(stderr, "-----------------------------------\n");
      fprintf(stderr, "Searching edge #%d in layer %d, head %d, tail %d\n",
              ei->ID() / 2, l, h, t);
    }
  }
  else
  {
    if (terminal_output_)
    {
      fprintf(stderr, "-----------------------------------\n");
      fprintf(stderr, "Searching starts in layer %d, head %d, tail %d\n", l, h, t);
    }
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
    double cost = GenerateCost(ej, h, t, l);

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
  WF_edge* prev_e = NULL;

  /* ranked by weight */
  for (it = choice.begin(); it != choice.end(); it++)
  {
    if(print_queue_.size() > 0)
    {
      prev_e = print_queue_.back();
    }

//    WF_edge* ej = it->second;

    WF_edge* ej = RouteEdgeDirection(prev_e, it->second);
    print_queue_.push_back(ej);

    /* update printed subgraph */
    UpdateStructure(ej, update_collision_);

    // update collision (geometric domain)
    // tmp is the pruned domain by direct arc consistency pruning
    std::map<int, EEDirArray> tmp_cmap;
    UpdateStateMap(ej, tmp_cmap);

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
    RecoverStateMap(ej, tmp_cmap);
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


double FFAnalyzer::GenerateCost(WF_edge *ej, const int h, const int t, const int l)
{
  const int orig_j = ej->ID();
  const int dual_j = ptr_wholegraph_->e_dual_id(orig_j);

  // check arc consistency for all unprinted elements
  if (!ptr_dualgraph_->isExistingEdge(ej))
  {
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

    // prune floating edge
    if(!ej->isPillar())
    {
      if (exist_uj && exist_vj)
      {
      }
      else
      {
        if (exist_uj || exist_vj)
        {
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
    }

    /* collision test */
    if(keep_timing_)
    {
      test_collision_.Start();
    }

    // TODO: just do a summation would be suffice
    int free_angle = std::accumulate(ee_dir_states_[dual_j].begin(), ee_dir_states_[dual_j].end(), 0);

    if(keep_timing_)
    {
      test_collision_.Stop();
    }

    if (free_angle == 0)
    {
      if (terminal_output_)
      {
        fprintf(stderr, "...collision test failed at edge #%d.\n", ej->ID() / 2);

        fprintf(stderr,
                "fail edge vert u: (%f, %f, %f), vert v: (%f, %f, %f)\n\n",
                ej->pvert_->Position().x(),
                ej->pvert_->Position().y(),
                ej->pvert_->Position().z(),
                ej->ppair_->pvert_->Position().x(),
                ej->ppair_->pvert_->Position().y(),
                ej->ppair_->pvert_->Position().z());
      }

      return -2;
    }
    else
    {
      /* robot kinematics test */
      if(update_collision_)
      {
        if (!TestRobotKinematics(ej, ee_dir_states_[dual_j]))
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
    if (!ej->isPillar() && !TestifyStiffness(ej))
    {
      /* examination failed */
      if (terminal_output_)
      {
        fprintf(stderr, "...stiffness examination failed at edge #%d.\n\n", ej->ID() / 2);
      }
//      return -1;
    }

    // Forward Checking
    // limit forward checking only to current layer
    /* influence weight */
    bool use_fc = 1;

    if(use_fc)
    {
      // consider only edges in the same layer
      int Nl = layers_[l].size();
      int remaining_num = 0;
      double forward_checking_factor = 1.0;

      if (keep_timing_)
      {
        gen_cost_fc_.Start();
      }
      for (int k = 0; k < Nl; k++)
      {
        WF_edge *ek = layers_[l][k];
        int dual_k = ptr_wholegraph_->e_dual_id(ek->ID());

        // considered edge is not ej itself and it's not printed yet
        if (dual_j != dual_k && !ptr_dualgraph_->isExistingEdge(ek))
        {
          EEDirArray tmp_cmap;
          ptr_collision_->DetectCollision(ek, ej, ee_dir_states_[dual_k], tmp_cmap);

          int future_angle = std::accumulate(tmp_cmap.begin(), tmp_cmap.end(), 0);

          if (0 == future_angle)
          {
            if (terminal_output_)
            {
              fprintf(stderr, "...FC pruning - collision test failed at edge #%d to future edge #%d.\n\n",
                  ej->ID() / 2, ek->ID() / 2);
            }
            return -1;
          }

          double d_ratio = (double) future_angle / (double) DIR_SPHERE_DIVISION;
          I += exp(-forward_checking_factor * d_ratio * d_ratio);

          remaining_num++;
        }
      }
      if (keep_timing_)
      {
        gen_cost_fc_.Stop();
      }

      if (0 != remaining_num)
      {
        I /= remaining_num;
      }
      else
      {
        I = 0.0;
      }
    }

    /* adjacency weight (A) */
    if(ej->isPillar())
    {
      // use dist to robot base as heuristic
      const auto &st_pt_msg = frame_msgs_[ej->ID()].start_pt;
      const auto &end_pt_msg = frame_msgs_[ej->ID()].end_pt;

      // distance to robot base (world_frame 0,0,0)
      double dist = sqrt(pow((st_pt_msg.x + end_pt_msg.x) / 2, 2)
                             + pow((st_pt_msg.y + end_pt_msg.y) / 2, 2)
                             + pow((st_pt_msg.z + end_pt_msg.z) / 2, 2));
      if (min_base_dist_ != max_base_dist_)
      {
        A = 1 - (dist - min_base_dist_) / (max_base_dist_ - min_base_dist_);
      }
    }

    double cost = Wa_ * A + Wi_ * I;

    if (terminal_output_)
    {
      fprintf(stderr, "A: %lf, I: %lf\ncost: %f\n\n", A, I, cost);
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

  if (keep_timing_)
  {
    printf("\n");

    test_stiff_.Print("test stiffness:");;
    test_kin_.Print("test kinematics:");
    test_collision_.Print("test collision:");

    upd_frame_.Print("update frame:");
    retr_frame_.Print("retr frame:");

    upd_dir_map_.Print("update dir map:");
    retr_dir_map_.Print("retr dir map:");

    upd_collision_.Print("update collision:");
    retr_collision_.Print("retr collision:");

    gen_cost_fc_.Print("forward check:");

    printf("\n");
  }
  else
  {
    printf("***FFAnalyzer detailed timing turned off.\n");
  }
}
