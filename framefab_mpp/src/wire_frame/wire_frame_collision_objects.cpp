//
// Created by yijiangh on 4/29/17.
//

#include <boost/make_shared.hpp>

#include <wire_frame/wire_frame_collision_objects.h>


namespace framefab
{
namespace wire_frame
{

WireFrameCollisionObjects::WireFrameCollisionObjects() : WireFrameLineGraph()
{
  ptr_vert_collision_objects_list_ = boost::make_shared<MoveitLinearMemberCollisionObjectsList>();
  ptr_edge_collision_objects_list_ = boost::make_shared<MoveitLinearMemberCollisionObjectsList>();
}

void WireFrameCollisionObjects::constructCollisionObjects(
    const planning_scene_monitor::PlanningSceneMonitorConstPtr ptr_planning_scene_monitor,
    const int pwf_scale_factor, const double display_point_radius)
{
  int m = pedge_list_->size();
  int n = pvert_list_->size();
  std::string frame_id = ptr_planning_scene_monitor->getPlanningScene()->getPlanningFrame();

  for(int i=0; i < m; i++)
  {
    MoveitLinearMemberCollisionObjectsPtr ptr_member_collision_obj =
        boost::make_shared<MoveitLinearMemberCollisionObjects>();

    ptr_member_collision_obj->edge_cylinder_collision = boost::make_shared<MoveitCollisionObject>();
    ptr_member_collision_obj->start_vertex_collision  = boost::make_shared<MoveitCollisionObject>();
    ptr_member_collision_obj->end_vertex_collision    = boost::make_shared<MoveitCollisionObject>();

    std::string cylinder_id = "cylinder-" + std::to_string(i);

    // get WF_edge
    WF_edge* ptr_wf_e = GetEdge(i);
    int start_vertex_id_u = ptr_wf_e->ppair_->pvert_->ID();
    int end_vertex_id_v = ptr_wf_e->pvert_->ID();

    // cylinder setup
    ptr_member_collision_obj->edge_cylinder_collision->id = cylinder_id;
    ptr_member_collision_obj->edge_cylinder_collision->header.frame_id = frame_id;
    ptr_member_collision_obj->edge_cylinder_collision->operation
        = moveit_msgs::CollisionObject::ADD;

    shape_msgs::SolidPrimitive cylinder_sollid;
    cylinder_sollid.type = shape_msgs::SolidPrimitive::CYLINDER;
    cylinder_sollid.dimensions.resize(2);
    cylinder_sollid.dimensions[0] = pwf_scale_factor * ptr_wf_e->Length();
    cylinder_sollid.dimensions[1] = display_point_radius;

    // start node setup
    std::string start_vertex_id = "start_vertex-" + std::to_string(start_vertex_id_u);
    ptr_member_collision_obj->start_vertex_collision->id = start_vertex_id;
    ptr_member_collision_obj->start_vertex_collision->header.frame_id = frame_id;
    ptr_member_collision_obj->start_vertex_collision->operation
        = moveit_msgs::CollisionObject::ADD;


  }


}

}// namespace wire_frame
}// namespace framefab
