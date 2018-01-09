#include <ros/console.h>

#include "framefab_task_sequence_planner/sequence_analyzers/SeqAnalyzer.h"

// planning scene
#include <moveit/planning_scene/planning_scene.h>

// msgs
#include <moveit_msgs/CollisionObject.h>

// msg-eigen conversion
#include <eigen_conversions/eigen_msg.h>

const static double ROBOT_KINEMATICS_CHECK_TIMEOUT = 10.0;

namespace{
// copy from graph_builder.cpp
// Generate evenly sampled point at some discretization 'ds' between start and stop.
// ds must be > 0
std::vector<Eigen::Vector3d> discretizePositions(const Eigen::Vector3d& start, const Eigen::Vector3d& stop, const double ds)
{
  double dist = (stop - start).norm();

  size_t n_intermediate_points = 0;
  if (dist > ds)
  {
    n_intermediate_points = static_cast<size_t>(std::lround(dist / ds));
  }

  const auto total_points = 2 + n_intermediate_points;

  std::vector<Eigen::Vector3d> result;
  result.reserve(total_points);

  for (std::size_t i = 0; i < total_points; ++i)
  {
    const double r = i / static_cast<double>(total_points - 1);
    Eigen::Vector3d point = start + (stop - start) * r;
    result.push_back(point);
  }
  return result;
}

// copy from descartes_planner::graph_builder.cpp
Eigen::Affine3d makePose(const Eigen::Vector3d& position, const Eigen::Matrix3d& orientation,
                         const double z_axis_angle)
{
  Eigen::Affine3d m = Eigen::Affine3d::Identity();
  m.matrix().block<3,3>(0,0) = orientation;
  m.matrix().col(3).head<3>() = position;

  Eigen::AngleAxisd z_rot (z_axis_angle, Eigen::Vector3d::UnitZ());

  return m * z_rot;
}

// copy from descartes_planner::capsulated_ladder_tree_RRTstar
static int randomSampleInt(int lower, int upper)
{
  std::random_device rd;
  std::mt19937 gen(rd());

  if(upper > lower)
  {
    std::uniform_int_distribution<int> int_distr(0, upper);
    return int_distr(gen);
  }
  else
  {
    return lower;
  }
}

static double randomSampleDouble(double lower, double upper)
{
  std::random_device rd;
  std::mt19937 gen(rd());

  if(upper > lower)
  {
    std::uniform_real_distribution<double> double_distr(lower, upper);
    return double_distr(gen);
  }
  else
  {
    return lower;
  }
}

std::vector<Eigen::Affine3d> generateSampleEEFPoses(
    const std::vector<Eigen::Vector3d>& path_pts,
    const std::vector<Eigen::Matrix3d>& direction_list)
{
  // sample int for orientation
  int o_sample = randomSampleInt(0, direction_list.size()-1);
  Eigen::Matrix3d orientation_sample = direction_list[o_sample];

  // sample [0,1] for axis, z_axis_angle = b_rand * 2 * Pi
  double x_axis_sample = randomSampleDouble(0.0, 1.0) * 2 * M_PI;

  std::vector<Eigen::Affine3d> poses;
  poses.reserve(path_pts.size());

  for(const auto& pt : path_pts)
  {
    poses.push_back(makePose(pt, orientation_sample, x_axis_sample));
  }

  return poses;
}

void convertOrientationVector(
    const std::vector<Eigen::Vector3d>& vec_orients,
    std::vector<Eigen::Matrix3d>& m_orients)
{
  m_orients.clear();

  for(auto eigen_vec : vec_orients)
  {
    // eigen_vec = local z axis
    eigen_vec *= -1.0;
    eigen_vec.normalize();

    // construct local x axis & y axis
    Eigen::Vector3d candidate_dir = Eigen::Vector3d::UnitX();
    if ( std::abs(eigen_vec.dot(Eigen::Vector3d::UnitX())) > 0.8 )
    {
      // if z axis = UnitX,
      candidate_dir = Eigen::Vector3d::UnitY();
    }

    Eigen::Vector3d y_vec = eigen_vec.cross(candidate_dir).normalized();

    Eigen::Vector3d x_vec = y_vec.cross(eigen_vec).normalized();

    // JM
    Eigen::Matrix3d m = Eigen::Matrix3d::Identity();
    m.col(0) = x_vec;
    m.col(1) = y_vec;
    m.col(2) = eigen_vec;

    m_orients.push_back(m);
  }
}

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

      // TODO: enable collision checking and greedy tsp here
    }
  }

  for (it = base_queue.begin(); it != base_queue.end(); it++)
  {
    print_queue_.push_back(it->second);
  }

  if (terminal_output_)
  {
    fprintf(stderr, "Size of base queue: %d\n", (int)base_queue.size());
  }

  /* angle state with pillars */
  for (int dual_i = 0; dual_i < Nd_; dual_i++)
  {
    WF_edge* e = ptr_frame_->GetEdge(ptr_wholegraph_->e_orig_id(dual_i));

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

    // for each unprinted edge in wireframe
    if (dual_i != dual_j && !ptr_dualgraph_->isExistingEdge(target_e))
    {
//      if (terminal_output_)
//      {
//        upd_map_collision_.Start();
//      }

      // prune order_e's domain with target_e's existence
      // arc consistency pruning
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

  for(int dual_j = 0; dual_j < Nd; dual_j++)
  {
    WF_edge * target_e = ptr_frame_->GetEdge(ptr_wholegraph_->e_orig_id(dual_j));

    if(dual_i != dual_j && !ptr_dualgraph_->isExistingEdge(target_e))
    {
      for(int k = 0; k < 3; k++)
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
  if (e->ID() < e->ppair_->ID())
  {
    orig_j = e->ID();
  }
  else
  {
    orig_j = e->ppair_->ID();
  }

  assert(orig_j < frame_msgs_.size());

  if(!shrink)
  {
    // add input edge's collision object
    moveit_msgs::CollisionObject e_collision_obj;
    e_collision_obj = frame_msgs_[orig_j].full_collision_object;
    e_collision_obj.operation = moveit_msgs::CollisionObject::ADD;

    // add this edge to the planning scene
    if (!planning_scene_->processCollisionObjectMsg(e_collision_obj))
    {
      ROS_WARN_STREAM("[ts planning] Failed to add shrinked collision object: edge #" << orig_j);
    }
  }
  else
  {
    // st node index
    int uj = ptr_frame_->GetEndu(orig_j);
    bool exist_uj = ptr_dualgraph_->isExistingVert(uj);

    // end node index
    int vj = ptr_frame_->GetEndv(orig_j);
    bool exist_vj = ptr_dualgraph_->isExistingVert(vj);

    std::vector<int> shrink_vert_ids;

    if (exist_uj)
    {
      shrink_vert_ids.push_back(uj);
    }

    if (exist_vj)
    {
      shrink_vert_ids.push_back(vj);
    }

    // query connected & existing edges for these nodes, shrink them
    for (const auto &connect_vert_id : shrink_vert_ids)
    {
      WF_edge *eu = ptr_frame_->GetNeighborEdge(connect_vert_id);

      while (eu != NULL)
      {
        if(eu->ID() == e->ID() || eu->ID() == e->ppair_->ID()
            || !ptr_dualgraph_->isExistingEdge(eu))
        {
          // skip if edge = input edge OR current edge doesn't exist yet
          eu = eu->pnext_;
          continue;
        }

        // pop the neighnor edge from existing planning scene
        int ne_id = eu->ID();
        moveit_msgs::CollisionObject ne_collision_obj;

        // replace with shrinked ones
        if(connect_vert_id == eu->ppair_->pvert_->ID())
        {
          ne_collision_obj = frame_msgs_[ne_id].st_shrinked_collision_object;
        }

        if(connect_vert_id == eu->pvert_->ID())
        {
          ne_collision_obj = frame_msgs_[ne_id].end_shrinked_collision_object;
        }

        // Adds the object to the planning scene. If the object previously existed, it is replaced.
        ne_collision_obj.operation = moveit_msgs::CollisionObject::ADD;

        if (!planning_scene_->processCollisionObjectMsg(ne_collision_obj))
        {
          ROS_WARN_STREAM("[ts planning] Update collision Obj:"
                              << " Failed to add collision object (shrinked neighnor): edge #" << orig_j);
        }

        eu = eu->pnext_;
      }
    }
  }
}

void SeqAnalyzer::RecoverCollisionObjects(WF_edge* e, bool shrink)
{
  int orig_j;
  if (e->ID() < e->ppair_->ID())
  {
    orig_j = e->ID();
  }
  else
  {
    orig_j = e->ppair_->ID();
  }

  assert(orig_j < frame_msgs_.size());

  if(!shrink)
  {
    // add input edge's collision object
    moveit_msgs::CollisionObject e_collision_obj;
    e_collision_obj = frame_msgs_[orig_j].full_collision_object;
    e_collision_obj.operation = moveit_msgs::CollisionObject::REMOVE;

    // add this edge to the planning scene
    if (!planning_scene_->processCollisionObjectMsg(e_collision_obj))
    {
      ROS_WARN_STREAM("[ts planning] Remove collision obj"
                          << "Failed to remove full collision object: edge #" << orig_j);
    }
  }
  else
  {
    // st node index
    int uj = ptr_frame_->GetEndu(orig_j);
    bool exist_uj = ptr_dualgraph_->isExistingVert(uj);

    // end node index
    int vj = ptr_frame_->GetEndv(orig_j);
    bool exist_vj = ptr_dualgraph_->isExistingVert(vj);

    std::vector<int> shrink_vert_ids;

    if (exist_uj)
    {
      shrink_vert_ids.push_back(uj);
    }

    if (exist_vj)
    {
      shrink_vert_ids.push_back(vj);
    }

    // query connected & existing edges for these nodes, shrink them
    for (const auto &connect_vert_id : shrink_vert_ids)
    {
      WF_edge *eu = ptr_frame_->GetNeighborEdge(connect_vert_id);

      while (eu != NULL)
      {
        if(eu->ID() == e->ID() || eu->ID() == e->ppair_->ID()
            || !ptr_dualgraph_->isExistingEdge(eu))
        {
          eu = eu->pnext_;
          continue;
        }

        // pop the neighnor edge from existing planning scene
        // Adds the object to the planning scene. If the object previously existed, it is replaced.
        int ne_id = eu->ID();
        moveit_msgs::CollisionObject ne_collision_obj;
        ne_collision_obj = frame_msgs_[ne_id].full_collision_object;
        ne_collision_obj.operation = moveit_msgs::CollisionObject::ADD;

        if (!planning_scene_->processCollisionObjectMsg(ne_collision_obj))
        {
          ROS_WARN_STREAM("[ts planning] Remove collision obj:"
                              << "Failed to replace collision object (shrinked neighnor): edge #" << orig_j);
        }

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

bool SeqAnalyzer::TestRobotKinematics(WF_edge *e, const std::vector<lld>& colli_map)
{
  // insert a trail edge, needs to shrink neighnoring edges
  // to avoid collision check between end effector and elements
  bool b_success = false;
  UpdateCollisionObjects(e, true);
  hotend_model_->setPlanningScene(planning_scene_);

//  std::ostream stream(nullptr);
//  std::stringbuf str;
//  stream.rdbuf(&str);
//  planning_scene_->printKnownObjects(stream);
//  ROS_INFO_STREAM("planning scene " << str.str());

  // generate feasible end effector directions for printing edge e
  std::vector<Eigen::Vector3d> direction_vec_list =
      ptr_collision_->ConvertCollisionMapToEigenDirections(colli_map);
  std::vector<Eigen::Matrix3d> direction_matrix_list;
  convertOrientationVector(direction_vec_list, direction_matrix_list);

  // generate eef path points
  Eigen::Vector3d st_pt;
  Eigen::Vector3d end_pt;

  tf::pointMsgToEigen(frame_msgs_[e->ID()].start_pt, st_pt);
  tf::pointMsgToEigen(frame_msgs_[e->ID()].end_pt, end_pt);

  std::vector<Eigen::Vector3d> path_pts = discretizePositions(st_pt, end_pt, 0.005);

  ROS_INFO_STREAM("path pts size: " << path_pts.size());

  const auto check_start_time = ros::Time::now();

  while((ros::Time::now() - check_start_time).toSec() < ROBOT_KINEMATICS_CHECK_TIMEOUT)
  {
    std::vector<Eigen::Affine3d> poses = generateSampleEEFPoses(path_pts, direction_matrix_list);

    bool empty_joint_pose_found = false;
    int cnt = 0;

    for(const auto& pose : poses)
    {
      std::vector<std::vector<double>> joint_poses;

      hotend_model_->getAllIK(pose, joint_poses);

      if(joint_poses.size() == 0)
      {
        empty_joint_pose_found = true;
        break;
      }
      else
      {
//        ROS_INFO_STREAM("non-zero jt pose found: " << cnt << "/" << poses.size());
        cnt++;
      }
    }

    if(!empty_joint_pose_found)
    {
      // all poses have feasible joint pose
      b_success = true;
      break;
    }
    else
    {
      b_success = false;
      continue;
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

void SeqAnalyzer::OutputTaskSequencePlanningResult(std::vector<SingleTaskPlanningResult>& planning_result)
{
  int Nq = print_queue_.size();
  planning_result.reserve(Nq);

  for (int i = 0; i < Nq; i++)
  {
    WF_edge* e = print_queue_[i];

    int dual_j = ptr_wholegraph_->e_dual_id(e->ID());

    std::vector<Eigen::Vector3d> direction_vec_list;

    if(e->isPillar())
    {
      direction_vec_list.push_back(Eigen::Vector3d(0,0,1));
    }
    else
    {
      assert(angle_state_[dual_j].size() > 0);
      // generate feasible end effector directions for printing edge e
      direction_vec_list = ptr_collision_->ConvertCollisionMapToEigenDirections(angle_state_[dual_j]);
    }

    assert(direction_vec_list.size() > 0);

    SingleTaskPlanningResult result;
    result.e_ = e;
    result.eef_directions_ = direction_vec_list;

    planning_result.push_back(result);
  }
}
