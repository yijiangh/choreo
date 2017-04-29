//
// Created by yijiangh on 4/29/17.
//
#include <assert.h>

#include <wire_frame/wire_frame_collision_objects.h>

namespace framefab
{
namespace wire_frame
{
WireFrameCollisionObjects::WireFrameCollisionObjects()
{
  ptr_vert_collision_obj_list_ = std::make_shared<WireFrameVertCollisionObjectsListPtr>();
  ptr_edge_collision_obj_list_ = std::make_shared<WireFrameEdgeCollisionObjectsListPtr>();
  ptr_wireframe_linegraph_ = std::make_shared<WireFrameLineGraphPtr>();
}

void WireFrameCollisionObjects::loadFromLineGraph(WireFrameCollisionObjects ptr_wire_frame_line_graph)
{
  std::weak_ptr<WireFrameLineGraph> ptr_weak_wf(ptr_wire_frame_line_graph);
  assert(ptr_weak_wf.lock());

  ptr_wire_frame_line_graph_ = ptr_wire_frame_line_graph;


}

}// namespace wire_frame
}// namespace framefab
