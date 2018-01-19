#include <ros/console.h>

#include "framefab_task_sequence_planner/FiberPrintPlugIn.h"

// moveit model
#include <moveit/robot_model_loader/robot_model_loader.h>

// msg
#include <framefab_msgs/ElementCandidatePoses.h>
#include <framefab_msgs/ModelInputParameters.h>
#include <framefab_msgs/TaskSequenceInputParameters.h>
#include <framefab_msgs/CollisionObjectList.h>

// srv
#include <moveit_msgs/ApplyPlanningScene.h>

// util
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

const static std::string COLLISION_OBJ_PREFIX = "wireframe_element";
const static std::string APPLY_PLANNING_SCENE_SERVICE = "apply_planning_scene";

namespace{
double dist(const Eigen::Vector3d& from, const Eigen::Vector3d& to)
{
  return (from - to).norm();
}

void createShrinkedEndPoint(Eigen::Vector3d& st_pt, Eigen::Vector3d& end_pt,
                            const double& shrink_length)
{
  Eigen::Vector3d translation_vec = end_pt - st_pt;
  translation_vec.normalize();

  st_pt = st_pt + shrink_length * translation_vec;
  end_pt = end_pt - shrink_length * translation_vec;
}

geometry_msgs::Pose computeCylinderPose(const Eigen::Vector3d& st_pt, const Eigen::Vector3d& end_pt)
{
  geometry_msgs::Pose cylinder_pose;

  // rotation
  Eigen::Vector3d axis = end_pt - st_pt;
  axis.normalize();
  Eigen::Vector3d z_vec(0.0, 0.0, 1.0);
  const Eigen::Vector3d& x_vec = axis.cross(z_vec);

  tf::Quaternion tf_q;
  if(0 == x_vec.norm())
  {
    // axis = z_vec
    tf_q = tf::Quaternion(0, 0, 0, 1);
  }
  else
  {
    double theta = axis.dot(z_vec);
    double angle = -1.0 * acos(theta);

    // convert eigen vertor to tf::Vector3
    tf::Vector3 x_vec_tf;
    tf::vectorEigenToTF(x_vec, x_vec_tf);

    // Create quaternion
    tf_q = tf::Quaternion(x_vec_tf, angle);
    tf_q.normalize();
  }

  //back to ros coords
  tf::quaternionTFToMsg(tf_q, cylinder_pose.orientation);
  tf::pointEigenToMsg((end_pt + st_pt) * 0.5, cylinder_pose.position);

  return cylinder_pose;
}

moveit_msgs::CollisionObject convertWFEdgeToCollisionObject(
    int edge_id, const Eigen::Vector3d st_pt, const Eigen::Vector3d end_pt, double element_diameter)
{
  moveit_msgs::CollisionObject collision_cylinder;
  std::string cylinder_id = COLLISION_OBJ_PREFIX + std::to_string(edge_id);

  // TODO: make frame_id as input parameter
  collision_cylinder.id = cylinder_id;
  collision_cylinder.header.frame_id = "world_frame";
  collision_cylinder.operation = moveit_msgs::CollisionObject::ADD;

  shape_msgs::SolidPrimitive cylinder_solid;
  cylinder_solid.type = shape_msgs::SolidPrimitive::CYLINDER;
  cylinder_solid.dimensions.resize(2);

  cylinder_solid.dimensions[0] = dist(st_pt, end_pt);

  cylinder_solid.dimensions[1] = element_diameter;
  collision_cylinder.primitives.push_back(cylinder_solid);
  collision_cylinder.primitive_poses.push_back(computeCylinderPose(st_pt, end_pt));

  return collision_cylinder;
}

void convertWireFrameToMsg(
    const framefab_msgs::ModelInputParameters& model_params,
    WireFrame& wire_frame, std::vector<framefab_msgs::ElementCandidatePoses>& wf_msgs,
    double& unit_scale)
{
  wf_msgs.clear();

  // set unit scale
  switch (model_params.unit_type)
  {
    case framefab_msgs::ModelInputParameters::MILLIMETER:
    {
      unit_scale = 0.001;
      break;
    }
    case framefab_msgs::ModelInputParameters::CENTIMETER:
    {
      unit_scale = 0.01;
      break;
    }
    case framefab_msgs::ModelInputParameters::INCH:
    {
      unit_scale = 0.0254;
      break;
    }
    case framefab_msgs::ModelInputParameters::FOOT:
    {
      unit_scale = 0.3048;
      break;
    }
    default:
    {
      ROS_ERROR("Unrecognized Unit type in Model Input Parameters!");
    }
  }

  // TODO: this might cause out-of-scale problem!!!
  // default wireframe is in millimeter and might be different to user-specified unit in mpp
  // wireframe default unit is mm, convert to meter
  const auto &base_pt = wire_frame.GetBaseCenterPos();
  Eigen::Vector3d transf_vec = Eigen::Vector3d(model_params.ref_pt_x * unit_scale - base_pt.x() * 0.001,
                                               model_params.ref_pt_y * unit_scale - base_pt.y() * 0.001,
                                               model_params.ref_pt_z * unit_scale - base_pt.z() * 0.001);

  wf_msgs.resize(wire_frame.SizeOfEdgeList());

  for (std::size_t i = 0; i < wire_frame.SizeOfEdgeList(); i++)
  {
    WF_edge *e = wire_frame.GetEdge(i);

    if (e->ID() < e->ppair_->ID())
    {
      framefab_msgs::ElementCandidatePoses element_msg;
      element_msg.element_id = e->ID();

      // notice these node positions are out-of-scale, default wireframe unit is millimeter
      // unit converted to meter
      Eigen::Vector3d eigen_st_pt(e->ppair_->pvert_->Position().x(),
                                  e->ppair_->pvert_->Position().y(),
                                  e->ppair_->pvert_->Position().z());
      eigen_st_pt = eigen_st_pt * 0.001 + transf_vec;

      Eigen::Vector3d eigen_end_pt(e->pvert_->Position().x(),
                                   e->pvert_->Position().y(),
                                   e->pvert_->Position().z());
      eigen_end_pt = eigen_end_pt * 0.001 + transf_vec;

      // in meter
      tf::pointEigenToMsg(eigen_st_pt, element_msg.start_pt);
      tf::pointEigenToMsg(eigen_end_pt, element_msg.end_pt);

      element_msg.element_diameter = model_params.element_diameter * unit_scale;

      element_msg.layer_id = e->Layer();

      // create collision objs
      Eigen::Vector3d shrinked_st_pt = eigen_st_pt;
      Eigen::Vector3d shrinked_end_pt = eigen_end_pt;
      createShrinkedEndPoint(shrinked_st_pt, shrinked_end_pt, model_params.shrink_length * unit_scale);

      element_msg.both_side_shrinked_collision_object = convertWFEdgeToCollisionObject(
          e->ID(), shrinked_st_pt, shrinked_end_pt, element_msg.element_diameter);

      element_msg.st_shrinked_collision_object = convertWFEdgeToCollisionObject(
          e->ID(), shrinked_st_pt, eigen_end_pt, element_msg.element_diameter);

      element_msg.end_shrinked_collision_object = convertWFEdgeToCollisionObject(
          e->ID(), eigen_st_pt, shrinked_end_pt, element_msg.element_diameter);

      element_msg.full_collision_object = convertWFEdgeToCollisionObject(
          e->ID(), eigen_st_pt, eigen_end_pt, element_msg.element_diameter);

      // TODO: this is redundant, a quick patch to make it work with wireframe's double-edge
      // data structure
      wf_msgs[e->ID()] = element_msg;
      wf_msgs[e->ppair_->ID()] = element_msg;
    }
  }
}

// TODO: temp functions
moveit_msgs::CollisionObject createPrintTable(const Eigen::Vector3d& ref_pt, const double unit_scale)
{
  // for now, only a simple flat box, representing the build plate, is added.
  // TODO: might need to use load mesh approach for user-customized scene collision setup
  // https://github.com/JeroenDM/descartes_tutorials/blob/indigo-devel/tutorial_utilities/src/collision_object_utils.cpp

  moveit_msgs::CollisionObject collision_env_obj;
  std::string env_obj_id = "env_obj_table";

  // table box's dimension
  double dx = 1;
  double dy = 1;
  double dz = 0.03;

  // pose
  Eigen::Affine3d rtn = Eigen::Translation3d(ref_pt[0] * unit_scale,
                                             ref_pt[1] * unit_scale,
                                             ref_pt[2] * unit_scale - dz/2)
      * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX())
      * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY())
      * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ());
  geometry_msgs::Pose pose;
  tf::poseEigenToMsg(rtn, pose);

  collision_env_obj.id = env_obj_id;
  collision_env_obj.header.frame_id = "world_frame";
  collision_env_obj.operation = moveit_msgs::CollisionObject::ADD;

  shape_msgs::SolidPrimitive env_obj_solid;
  env_obj_solid.type = shape_msgs::SolidPrimitive::BOX;
  env_obj_solid.dimensions.resize(3);
  env_obj_solid.dimensions[0] = dx;
  env_obj_solid.dimensions[1] = dy;
  env_obj_solid.dimensions[2] = dz;
  collision_env_obj.primitives.push_back(env_obj_solid);
  collision_env_obj.primitive_poses.push_back(pose);

  return collision_env_obj;
}

bool addCollisionObject(const moveit_msgs::CollisionObject& c_obj)
{
  ros::NodeHandle nh;
  ros::ServiceClient planning_scene_diff_client =
      nh.serviceClient<moveit_msgs::ApplyPlanningScene>(APPLY_PLANNING_SCENE_SERVICE);

  if(planning_scene_diff_client.waitForExistence())
  {
//    ROS_INFO_STREAM("planning scene diff srv connected!");
  }
  else
  {
    ROS_ERROR_STREAM("[ts planning] cannot connect with planning scene diff server...");
  }

  moveit_msgs::ApplyPlanningScene srv;

  moveit_msgs::PlanningScene planning_scene_msg;
  planning_scene_msg.world.collision_objects.push_back(c_obj);
  planning_scene_msg.is_diff = true;
  srv.request.scene = planning_scene_msg;

  if(planning_scene_diff_client.call(srv))
  {
//    ROS_INFO_STREAM("adding new collision object to planning scene published!");
    return true;
  }
  else
  {
    ROS_ERROR_STREAM("[ts planning] Failed to publish planning scene diff srv!");
    return false;
  }
}

}// util namespace

FiberPrintPlugIn::FiberPrintPlugIn(const std::string& world_frame,
                                   const std::string& hotend_group, const std::string& hotend_tcp,
                                   const std::string& robot_model_plugin)
    : plugin_loader_("descartes_core", "descartes_core::RobotModel"),
      hotend_group_name_(hotend_group)
{
  ptr_frame_ = NULL;
  ptr_dualgraph_ = NULL;
  ptr_collision_ = NULL;
  ptr_stiffness_ = NULL;

  ptr_seqanalyzer_ = NULL;
  ptr_procanalyzer_ = NULL;

  ptr_path_ = NULL;
  ptr_parm_ = NULL;

  terminal_output_ = false;
  file_output_ = false;

  // Attempt to load and initialize the printing robot model (hotend)
  hotend_model_ = plugin_loader_.createInstance(robot_model_plugin);
  if (!hotend_model_)
  {
    throw std::runtime_error(std::string("Could not load: ") + robot_model_plugin);
  }

  if (!hotend_model_->initialize("robot_description", hotend_group, world_frame, hotend_tcp))
  {
    throw std::runtime_error("Unable to initialize printing robot model");
  }

  // Load the moveit model
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  moveit_model_ = robot_model_loader.getModel();

  if (moveit_model_.get() == NULL)
  {
    throw std::runtime_error("Could not load moveit robot model");
  }

  // Enable Collision Checks
  hotend_model_->setCheckCollisions(true);
}

FiberPrintPlugIn::~FiberPrintPlugIn()
{
  delete ptr_dualgraph_;
  ptr_dualgraph_ = NULL;

  delete ptr_collision_;
  ptr_collision_ = NULL;

  delete ptr_stiffness_;
  ptr_stiffness_ = NULL;

  delete ptr_seqanalyzer_;
  ptr_seqanalyzer_ = NULL;

  delete ptr_procanalyzer_;
  ptr_procanalyzer_ = NULL;

  delete ptr_parm_;
  ptr_parm_ = NULL;

  delete ptr_frame_;
  ptr_frame_ = NULL;
}

bool FiberPrintPlugIn::Init()
{
  if(ptr_frame_ != NULL && ptr_parm_ != NULL && ptr_path_ != NULL)
  {
    delete ptr_dualgraph_;
    ptr_dualgraph_ = new DualGraph(ptr_frame_);

    delete ptr_collision_;
    ptr_collision_ = new QuadricCollision(ptr_frame_);

    delete ptr_stiffness_;
    ptr_stiffness_ = new Stiffness(
        ptr_dualgraph_, ptr_parm_,
        ptr_path_, false);

    delete ptr_seqanalyzer_;
    ptr_seqanalyzer_ = NULL;

    delete ptr_procanalyzer_;
    ptr_procanalyzer_ = NULL;

    return true;
  }
  else
  {
    return false;
  }
}

bool FiberPrintPlugIn::DirectSearch()
{
  fiber_print_.Start();

  if(Init())
  {
    assert(ptr_frame_->SizeOfEdgeList() == frame_msgs_.size());

    ptr_seqanalyzer_ = new GecodeAnalyzer(
        ptr_dualgraph_,
        ptr_collision_,
        ptr_stiffness_,
        ptr_parm_,
        ptr_path_,
        terminal_output_,
        file_output_,
        hotend_model_,
        moveit_model_,
        hotend_group_name_
    );

    ptr_seqanalyzer_->setFrameMsgs(frame_msgs_);

    if (!ptr_seqanalyzer_->SeqPrint())
    {
      ROS_WARN("Model not printable!");
      return false;
    }

    fiber_print_.Stop();
    fiber_print_.Print("Direct Search:");

    return true;
  }
  else
  {
    ROS_WARN_STREAM("[ts planning] wireframe, fiber_print parm or output_path not initiated."
                        << "ts planning failed.");
    return false;
  }
}

bool FiberPrintPlugIn::ConstructCollisionObjects(const std::vector<int>& print_queue_edge_ids,
                                                 std::vector<framefab_msgs::WireFrameCollisionObject>& collision_objs)
{
  if(Init())
  {
    assert(ptr_frame_->SizeOfEdgeList() == frame_msgs_.size());

    ptr_seqanalyzer_ = new FFAnalyzer(
        ptr_dualgraph_,
        ptr_collision_,
        ptr_stiffness_,
        ptr_parm_,
        ptr_path_,
        terminal_output_,
        file_output_,
        hotend_model_,
        moveit_model_,
        hotend_group_name_
    );

    ptr_seqanalyzer_->setFrameMsgs(frame_msgs_);

    return ptr_seqanalyzer_->ConstructCollisionObjsInQueue(print_queue_edge_ids, collision_objs);
  }
  else
  {
    ROS_WARN_STREAM("[ts planning] wireframe, fiber_print parm or output_path not initiated."
                        << "constructing collision objs in ts planning failed.");
    return false;
  }
}

void FiberPrintPlugIn::GetDeformation()
{
  ptr_dualgraph_->Dualization();

  VX D;
  ptr_stiffness_->Init();
  ptr_stiffness_->CalculateD(D, NULL, false, 0, "FiberTest");
}

bool FiberPrintPlugIn::handleTaskSequencePlanning(
    framefab_msgs::TaskSequencePlanning::Request& req,
    framefab_msgs::TaskSequencePlanning::Response& res)
{
  switch(req.action)
  {
    case framefab_msgs::TaskSequencePlanning::Request::READ_WIREFRAME:
    {
      std::string file_path = req.model_params.file_name;

      // TODO: all of these char* should be const char*
      // convert std::string to writable char*
      std::vector<char> fp(file_path.begin(), file_path.end());
      fp.push_back('\0');

      if(NULL != ptr_frame_)
      {
        delete ptr_frame_;
      }

      // TODO: if contains keyword "pwf"
      ptr_frame_ = new WireFrame();
      ptr_frame_->LoadFromPWF(&fp[0]);

      double unit_scale;
      Eigen::Vector3d ref_pt(req.model_params.ref_pt_x, req.model_params.ref_pt_y, req.model_params.ref_pt_z);

      // TODO: temp functions
      convertWireFrameToMsg(req.model_params, *ptr_frame_, frame_msgs_, unit_scale);

      moveit_msgs::CollisionObject table = createPrintTable(ref_pt, unit_scale);
      addCollisionObject(table);

      res.element_array = frame_msgs_;

      break;
    }
    case framefab_msgs::TaskSequencePlanning::Request::TASK_SEQUENCE_SEARCHING:
    {
      double Wp = 1.0;
      double Wa = 1.0;
      double Wi = 3.0;

      if(NULL != ptr_parm_)
      {
        delete ptr_parm_;
      }

      ptr_parm_ = new FiberPrintPARM(Wp, Wa, Wi);

      // dummy framefab output path
      ptr_path_ = "/home";
      const char* json_output_path = req.task_sequence_params.file_path.c_str();

      terminal_output_ = true;

      if(DirectSearch())
      {
        assert(NULL != ptr_seqanalyzer_);

        ptr_procanalyzer_ = new ProcAnalyzer(ptr_seqanalyzer_, json_output_path);

        if(!ptr_procanalyzer_->ProcPrint())
        {
          ROS_ERROR_STREAM("[ts planner] proc analyzer failed.");
          return false;
        }
      }
      else
      {
        ROS_ERROR_STREAM("[ts planner] direct searching failed.");
        return false;
      }

      break;
    }
    case framefab_msgs::TaskSequencePlanning::Request::REQUEST_COLLISION_OBJS:
    {
      if(NULL != ptr_parm_)
      {
        delete ptr_parm_;
      }

      // dummy parm
      ptr_parm_ = new FiberPrintPARM(1,0, 1.0, 1.0);

      // dummy framefab output path
      ptr_path_ = "/home";

      terminal_output_ = true;

      // construct wireframe ids in the queue
      std::vector<int> wireframe_ids;

      for(const auto& element : req.element_array)
      {
        wireframe_ids.push_back(element.wireframe_id);
      }

      if(!ConstructCollisionObjects(wireframe_ids, res.wf_collision_objs))
      {
        ROS_ERROR_STREAM("[ts planner] construct collision objects failed.");
        return false;
      }

      break;
    }
    default:
    {
      ROS_ERROR_STREAM("[ts planning] unknown task sequence planning request action.");
      return false;
    }
  }

  return true;
}
