//
// Created by yijiangh on 11/7/18.
//

#include "choreo_task_sequence_planner/sequence_analyzers/FFAnalyzer.h"

bool FFAnalyzer::SeqPrintLayer(std::vector<int> layer_ids)
{
//  assert(ptr_frame_->SizeOfLayer()
//             > *std::max_element(layer_ids.begin(), layer_ids.end()));
//
//  ROS_INFO_STREAM("[TSP] Target layers search test.");
//
//  FF_analyzer_.Start();
//
//  Init();
//
//  ROS_INFO_STREAM("[TSP] FF Analyzer init.");
//
//  /* split layers */
//  /* label stores layer index of each dual node */
//  int layer_size = ptr_frame_->SizeOfLayer();
//  layers_.clear();
//  layers_.resize(layer_size);
//  for (int i = 0; i < Nd_; i++)
//  {
//    WF_edge *e = ptr_frame_->GetEdge(ptr_wholegraph_->e_orig_id(i));
//    assert(e);
//    layers_[e->Layer()].push_back(e);
//  }
//
//  for(const int& layer_id : layer_ids)
//  {
//    fprintf(stderr, "Size of target layer %d is %d\n", layer_id, (int)layers_[layer_id].size());
//  }
//
//  int min_tid = *std::min_element(layer_ids.begin(), layer_ids.end());
//
//  bool bSuccess = true;
//
//  for(int l=0; l < min_tid; l++)
//  {
//    std::cout << "layer #" << l << std::endl;
//
//    for(int s=0; s < layers_[l].size(); s++)
//    {
//      WF_edge* e = layers_[l][s];
//
//      print_queue_.push_back(e);
//
//      // update printed graph
//      UpdateStructure(e, update_collision_);
//
//      for(const int& layer_id : layer_ids)
//      {
//        vector <vector<lld>> state_map(3);
//        int dual_i = ptr_wholegraph_->e_dual_id(e->ID());
//
//        for (int j = 0; j < layers_[layer_id].size(); j++)
//        {
//          WF_edge* target_e = layers_[layer_id][j];
//          int dual_j = ptr_wholegraph_->e_dual_id(target_e->ID());
//
//          // prune order_e's domain with target_e's existence
//          // arc consistency pruning
//          vector <lld> tmp(3);
//          if (ptr_collision_->DetectCollision(target_e, e, tmp))
//          {
//            for (int k = 0; k < 3; k++)
//            {
//              state_map[k].push_back(ee_dir_states_[dual_j][k]);
//            }
//
//            ptr_collision_->ModifyAngle(ee_dir_states_[dual_j], tmp);
//          }
//        }
//      } // loop thru target layers
//    }
//  }
//
//  std::cout << "Update finished." << std::endl;
//
//  int target_size = 0;
//
//  for(const int& lb : layer_ids)
//  {
//    // search in target layer
//    int Nl = layers_[lb].size();
//    int h = print_queue_.size(); // print queue size so far
//    int t = h + Nl;
//
//    target_size += Nl;
//
//    /* max_z_ and min_z_ in current layer */
//    min_base_dist_ = min_z_ = 1e20;
//    max_base_dist_ = max_z_ = -1e20;
//
//    for (int i = 0; i < Nl; i++)
//    {
//      WF_edge *e = layers_[lb][i];
//      point u = e->pvert_->Position();
//      point v = e->ppair_->pvert_->Position();
//      min_z_ = min(min_z_, (double) min(u.z(), v.z()));
//      max_z_ = max(max_z_, (double) max(u.z(), v.z()));
//
//      const auto& st_pt_msg = frame_msgs_[e->ID()].start_pt;
//      const auto& end_pt_msg = frame_msgs_[e->ID()].end_pt;
//
//      // distance to robot base (world_frame 0,0,0)
//      double dist = sqrt(pow((st_pt_msg.x + end_pt_msg.x)/2, 2)
//                             + pow((st_pt_msg.y + end_pt_msg.y)/2, 2)
//                             + pow((st_pt_msg.z + end_pt_msg.z)/2, 2));
//
//      min_base_dist_ = min(min_base_dist_, dist);
//      max_base_dist_ = max(max_base_dist_, dist);
//    }
//
//    search_rerun_ = 0;
//    bSuccess = false;
//
//    while(!bSuccess && search_rerun_ < 2)
//    {
//      bSuccess = GenerateSeq(lb, h, t);
//      search_rerun_++;
//    }
//
//    if(!bSuccess)
//    {
//      ROS_ERROR("All possible start edge at layer %d has been tried but no feasible sequence is obtained after %d iterations.",
//                lb, search_rerun_);
//      break;
//    }
//  }
//
//  fprintf(stderr, "****Reminder: Start of target layers is %d\n", Nd_ - target_size);
//
//  FF_analyzer_.Stop();
//  return bSuccess;
}
