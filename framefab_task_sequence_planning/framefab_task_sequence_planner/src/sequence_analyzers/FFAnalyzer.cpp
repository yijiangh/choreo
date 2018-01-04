#include "framefab_task_sequence_planner/sequence_analyzers/FFAnalyzer.h"

//FFAnalyzer::FFAnalyzer(WireFrame *ptr_frame)
//{
//  ptr_frame_ = ptr_frame;
//}

FFAnalyzer::~FFAnalyzer()
{
}

bool FFAnalyzer::SeqPrint()
{
  FF_analyzer_.Start();

  Init();

  /* printing */
  /* set pillars as starting edges */
  PrintPillars();

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

  for (int l = 0; l < layer_size; l++)
  {
    fprintf(stderr, "Size of layer %d is %d\n", l, layers_[l].size());
  }

  //Timer layer_search;

  /* print starting from the first layer */
  bool bSuccess = true;
  for (int l = 0; l < layer_size; l++)
  {
    /*
    * Nl: number of dual verts in current layer
    * h : head for printing queue of the layer
    * t : tail for printing queue of the layer
    */
    //layer_search.Reset();
    //layer_search.Start();

    int Nl = layers_[l].size();
    int h = print_queue_.size();
    int t;
    if (l == 0)
    {
      t = Nl;
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
    min_z_ = 1e20;
    max_z_ = -min_z_;
    for (int i = 0; i < Nl; i++)
    {
      WF_edge *e = layers_[l][i];
      point u = e->pvert_->Position();
      point v = e->ppair_->pvert_->Position();
      min_z_ = min(min_z_, (double)min(u.z(), v.z()));
      max_z_ = max(max_z_, (double)max(u.z(), v.z()));
    }

    if (!GenerateSeq(l, h, t))
    {
      fprintf(stderr,
              "All possible start edge at layer %d has been tried but no feasible sequence is obtained.\n",
              l + 1
      );
      bSuccess = false;
      break;
    }

    //layer_search.Stop();
    //
    //string str = std::to_string(l) + ":";
    //const char *msg = str.c_str();
    //char* cstr = new char[str.length() + 1];
    //strcpy(cstr, msg);

    //layer_search.Print(cstr);
    //printf("layer size: %d\n", Nl);
    //printf("layer %d finished\n", l);
    //printf("--------------\n");

    //getchar();
  }

  FF_analyzer_.Stop();
  return bSuccess;
}


bool FFAnalyzer::GenerateSeq(int l, int h, int t)
{
  /* last edge */
  assert(h != 0);						// there must be pillars
  WF_edge *ei = print_queue_[h - 1];

  if (terminal_output_)
  {
    fprintf(stderr, "-----------------------------------\n");
    fprintf(stderr, "Searching edge #%d in layer %d, head %d, tail %d\n",
            ei->ID() / 2, l + 1, h, t);
  }

  /* exit */
  if (h == t)
  {
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
    double cost = GenerateCost(ei, ej);
    if (cost != -1)
    {
      // eligible candidate, cost = -1 if stiffness or kinematic check not pass
      choice.insert(pair<double, WF_edge*>(cost, ej));
    }
  }

  /* ranked by weight */
  for (it = choice.begin(); it != choice.end(); it++)
  {
    WF_edge *ej = it->second;
    print_queue_.push_back(ej);

    /* update printed subgraph */
    UpdateStructure(ej, update_collision_);

    /* update collision (geometric domain) */
    vector<vector<lld>> tmp_angle(3);
    UpdateStateMap(ej, tmp_angle);

    if(terminal_output_)
    {
      fprintf(stderr, "Choose edge #%d in with cost %lf\n\n", ej->ID() / 2, it->first);
    }

    if(GenerateSeq(l, h + 1, t))
    {
      return true;
    }

    // backtrack
    RecoverStateMap(ej, tmp_angle);
    RecoverStructure(ej, update_collision_);

    print_queue_.pop_back();
  }

  return false;
}


double FFAnalyzer::GenerateCost(WF_edge *ei, WF_edge *ej)
{
  int orig_j = ej->ID();
  int dual_j = ptr_wholegraph_->e_dual_id(orig_j);

  // check arc consistency for all unprinted elements
  if (!ptr_dualgraph_->isExistingEdge(ej))
  {
    double	P = 0;							// stabiliy weight
    double  A = 0;							// adjacency weight
    double	I = 0;							// influence weight

    if (terminal_output_)
    {
      fprintf(stderr, "Attempting edge #%d\n",
              orig_j / 2, ej->Layer() + 1, print_queue_.size());
    }

    /* stabiliy weight */
    int uj = ptr_frame_->GetEndu(orig_j);
    int vj = ptr_frame_->GetEndv(orig_j);
    bool exist_uj = ptr_dualgraph_->isExistingVert(uj);
    bool exist_vj = ptr_dualgraph_->isExistingVert(vj);
    double z = (ej->CenterPos().z() - min_z_) / (max_z_ - min_z_);

    // prune floating edge
    if (exist_uj && exist_vj)
    {
      /* edge j share two ends with printed structure */
//      if (terminal_output_)
//      {
//        fprintf(stderr, "It shares two ends with printed structure\n");
//      }
      P = z;
    }
    else
    if (exist_uj || exist_vj)
    {
      /* edge j share one end with printed structure */
//      if (terminal_output_)
//      {
//        fprintf(stderr, "It shares only one ends with printed structure\n");
//      }

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
        fprintf(stderr, "It floats, skip\n\n");
      }
      return -1;
    }

    /* collision test */
    int free_angle = ptr_collision_->ColFreeAngle(angle_state_[dual_j]);
    if (free_angle == 0)
    {
      if (terminal_output_)
      {
        fprintf(stderr, "...collision examination failed at edge #%d.\n\n", ej->ID() / 2);
      }
      return -1;
    }

    // TODO: if free directions' number < threshold, check kinematics
    /* robot kinematics test */
//    TestRobotKinematics();

    /* stiffness test */
    if (!TestifyStiffness(ej))
    {
      /* examination failed */
      if (terminal_output_)
      {
        fprintf(stderr, "Stiffness examination failed at edge #%d.\n\n", ej->ID() / 2);
      }
      return -1;
    }

    /* adjacency weight */
    if (ei == NULL)
    {
      A = 0;
    }
    else
    {
      int ui = ei->pvert_->ID();
      int vi = ei->ppair_->pvert_->ID();
      if (ui == uj || ui == vj || vi == uj || vi == vj)
      {
        A = 0;
      }
      else
      {
        A = 1.0;
      }
    }

    // TODO: should change into forward checking (one-step arc consistency)
    /* influence weight */
    int remaining = Nd_ - ptr_dualgraph_->SizeOfVertList();
    for (int dual_k = 0; dual_k < Nd_; dual_k++)
    {
      WF_edge *ek = ptr_frame_->GetEdge(ptr_wholegraph_->e_orig_id(dual_k));
      if (dual_j != dual_k && !ptr_dualgraph_->isExistingEdge(ek))
      {
        vector<lld> tmp(3);
        ptr_collision_->DetectCollision(ek, ej, tmp);
        for (int o = 0; o < 3; o++)
        {
          tmp[o] |= angle_state_[dual_k][o];
        }

        double tmp_range = ptr_collision_->ColFreeAngle(tmp) * 1.0 / ptr_collision_->Divide();
        I += exp(-5 * tmp_range * tmp_range);
      }
    }
    I /= remaining;

    double cost = Wp_*P + Wa_*A + Wi_*I;
    if (terminal_output_)
    {
      fprintf(stderr, "P: %lf, A: %lf, I: %lf\ncost: %f\n\n", P, A, I, cost);
    }
    return cost;
  }

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
