#include <ros/console.h>

#include "framefab_task_sequence_planner/sequence_analyzers/SeqAnalyzer.h"

// planning scene
#include <moveit/planning_scene/planning_scene.h>

// msgs
#include <moveit_msgs/CollisionObject.h>

const static double ROBOT_KINEMATICS_CHECK_TIMEOUT = 2.0;

namespace{
}// util namespace

SeqAnalyzer::SeqAnalyzer(
    DualGraph			*ptr_dualgraph,
    QuadricCollision	*ptr_collision,
    Stiffness			*ptr_stiffness,
    FiberPrintPARM		*ptr_parm,
    char				*ptr_path,
    bool				terminal_output,
    bool				file_output,
    descartes_core::RobotModelPtr hotend_model,
    moveit::core::RobotModelConstPtr moveit_model,
    std::string hotend_group_name
) noexcept
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

  update_collision_ = true;

  hotend_model_ = hotend_model;
  moveit_model_ = moveit_model;
  hotend_group_name_ = hotend_group_name;
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

  // init base planning scene
  planning_scene_ = planning_scene::PlanningScenePtr(new planning_scene::PlanningScene(moveit_model_));
  planning_scene_->getCurrentStateNonConst().setToDefaultValues();
  planning_scene_->getCurrentStateNonConst().update();
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

  if(update_collision)
  {
    // add full collision obj without shrinking
    UpdateCollisionObjects(e, false);
  }

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
}

void SeqAnalyzer::RecoverStructure(WF_edge *e, bool update_collision)
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

  if(update_collision)
  {
    // pop full collision obj without dealing with neighnoring edges
    RecoverCollisionObjects(e, false);
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

void SeqAnalyzer::UpdateCollisionObjects(WF_edge* e, bool shrink)
{
  int orig_j;
  if(e->ID() < e->ppair_->ID())
  {
    orig_j = e->ID();
  }
  else
  {
    orig_j = e->ppair_->ID();
  }

  assert(orig_j < frame_msgs_.size());
  moveit_msgs::CollisionObject e_collision_obj;

  // st node index
  int uj = ptr_frame_->GetEndu(orig_j);
  bool exist_uj = ptr_dualgraph_->isExistingVert(uj);

  // end node index
  int vj = ptr_frame_->GetEndv(orig_j);
  bool exist_vj = ptr_dualgraph_->isExistingVert(vj);

  std::vector<int> shrink_vert_ids;

  if(shrink || (exist_uj && exist_vj))
  {
    // both vertices exist, <connect> type, shrink both side
    e_collision_obj = frame_msgs_[orig_j].both_side_shrinked_collision_object;
    shrink_vert_ids.push_back(uj);
    shrink_vert_ids.push_back(vj);
  }
  else
  {
    // only one vertex exist, <create type>
    if(exist_uj)
    {
      e_collision_obj = frame_msgs_[orig_j].st_shrinked_collision_object;
      shrink_vert_ids.push_back(uj);
    }

    if(exist_vj)
    {
      e_collision_obj = frame_msgs_[orig_j].end_shrinked_collision_object;
      shrink_vert_ids.push_back(vj);
    }
  }

  // add this edge to the planning scene
  auto child_scene = planning_scene_->diff();

  if (!child_scene->processCollisionObjectMsg(e_collision_obj))
  {
    ROS_WARN_STREAM("[ts planning] Failed to add shrinked collision object: edge #" << orig_j);
  }

  if(shrink)
  {
    // query connected & existing edges for these nodes, shrink them
    for (const auto &connect_vert_id : shrink_vert_ids)
    {
      WF_edge *eu = ptr_frame_->GetNeighborEdge(connect_vert_id);

      while (eu != NULL)
      {
        // pop the neighnor edge from existing planning scene

        // replace with shrinked ones

        eu = eu->pnext_;
      }
    }
  }
}

void SeqAnalyzer::RecoverCollisionObjects(WF_edge* e, bool shrink)
{
  int orig_j;
  if(e->ID() < e->ppair_->ID())
  {
    orig_j = e->ID();
  }
  else
  {
    orig_j = e->ppair_->ID();
  }

  assert(orig_j < frame_msgs_.size());
  moveit_msgs::CollisionObject e_collision_obj;

  // st node index
  int uj = ptr_frame_->GetEndu(orig_j);
  bool exist_uj = ptr_dualgraph_->isExistingVert(uj);

  // end node index
  int vj = ptr_frame_->GetEndv(orig_j);
  bool exist_vj = ptr_dualgraph_->isExistingVert(vj);

  std::vector<int> shrink_vert_ids;

  if(shrink || (exist_uj && exist_vj))
  {
    // both vertices exist, <connect> type, shrink both side
    e_collision_obj = frame_msgs_[orig_j].both_side_shrinked_collision_object;
    shrink_vert_ids.push_back(uj);
    shrink_vert_ids.push_back(vj);
  }
  else
  {
    // only one vertex exist, <create type>
    if(exist_uj)
    {
      e_collision_obj = frame_msgs_[orig_j].st_shrinked_collision_object;
      shrink_vert_ids.push_back(uj);
    }

    if(exist_vj)
    {
      e_collision_obj = frame_msgs_[orig_j].end_shrinked_collision_object;
      shrink_vert_ids.push_back(vj);
    }
  }

  // add this edge to the planning scene
  auto child_scene = planning_scene_->diff();

  // TODO: pop collision obj
//  if (!child_scene->processCollisionObjectMsg(e_collision_obj))
//  {
//    ROS_WARN_STREAM("[ts planning] Failed to add shrinked collision object: edge #" << orig_j);
//  }

  if(shrink)
  {
    // query connected & existing edges for these nodes, shrink them
    for (const auto &connect_vert_id : shrink_vert_ids)
    {
      WF_edge *eu = ptr_frame_->GetNeighborEdge(connect_vert_id);

      while (eu != NULL)
      {
        // pop the neighnor edge from existing planning scene

        // replace with shrinked ones

        eu = eu->pnext_;
      }
    }
  }
}

bool SeqAnalyzer::TestifyStiffness(WF_edge *e)
{
//  if (terminal_output_)
//  {
//    test_stiff_.Start();
//  }

  /* insert a trail edge */
  UpdateStructure(e, false);

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
  RecoverStructure(e, false);

//  if (terminal_output_)
//  {
//    test_stiff_.Stop();
//  }

  return bSuccess;
}

bool SeqAnalyzer::TestRobotKinematics(WF_edge *e)
{
  // insert a trail edge, needs to shrink neighnoring edges
  // to avoid collision check between end effector and elements
  bool b_success = false;
  UpdateCollisionObjects(e, true);

  // RRT* improve on the tree
  const auto check_start_time = ros::Time::now();

  while((ros::Time::now() - check_start_time).toSec() < ROBOT_KINEMATICS_CHECK_TIMEOUT)
  {
    // make end effector pose
    // sample one direction from the maintained eef direction
    // sample one rotation angle from 2*pi
    // make eef pose

    std::vector <std::vector<double>> joint_poses;
//    hotend_model_.getAllIK(pose, joint_poses);

    if (joint_poses.empty())
    {
      b_success = false;
    }
    else
    {
      b_success = true;
      break;
    }
  }

  // remove the trail edge, change shrinked edges back to full collision objects
  RecoverCollisionObjects(e, true);
  return b_success;
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
