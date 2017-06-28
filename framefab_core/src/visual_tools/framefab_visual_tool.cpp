//
// Created by yijiangh on 6/27/17.
//

#include <framefab_core/visual_tools/framefab_visual_tool.h>

#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

void framefab_visual_tools::FrameFabVisualTool::init(std::string frame_name, std::string marker_topic)
{
  visual_tools_.reset(
      new rviz_visual_tools::RvizVisualTools(frame_name, marker_topic));

  // Clear messages
  visual_tools_->deleteAllMarkers();
  visual_tools_->enableBatchPublishing();
}

void framefab_visual_tools::FrameFabVisualTool::convertPathVisual(
    const PathArray& path_array, VisualPathArray& visual_path_array)
{
  visual_path_array.clear();

  for(int i=0; i < path_array_.size(); i++)
  {
    VisualUnitProcessPath v_unit_path;
    tf::pointMsgToEigen(path_array[i].shrinked_start_pt, v_unit_path.start_pt);
    tf::pointMsgToEigen(path_array[i].shrinked_end_pt, v_unit_path.end_pt);
    v_unit_path.type =
        static_cast<framefab_visual_tools::UNIT_PATH_TYPE>(path_array[i].type);
    v_unit_path.diameter = path_array[i].element_diameter;

    visual_path_array.push_back(v_unit_path);
  }
}

void framefab_visual_tools::FrameFabVisualTool::visualizeAllPaths()
{
  visual_tools_->deleteAllMarkers();

//  ROS_INFO_STREAM("publishing all paths...");

  for(int i=0; i < visual_path_array_.size(); i++)
  {
    rviz_visual_tools::colors type_color;

    if(framefab_visual_tools::UNIT_PATH_TYPE::SUPPORT ==  visual_path_array_[i].type)
    {
      type_color = rviz_visual_tools::BLUE;
    }

    if(framefab_visual_tools::UNIT_PATH_TYPE::CREATE ==  visual_path_array_[i].type)
    {
      type_color = rviz_visual_tools::YELLOW;
    }

    if(framefab_visual_tools::UNIT_PATH_TYPE::CONNECT ==  visual_path_array_[i].type)
    {
      type_color = rviz_visual_tools::RED;
    }

    visual_tools_->publishCylinder(visual_path_array_[i].start_pt,
                                   visual_path_array_[i].end_pt,
                                   type_color, visual_path_array_[i].diameter, "Cylinder");
  }

  visual_tools_->trigger();
}

void framefab_visual_tools::FrameFabVisualTool::visualizePath(int i)
{
  visual_tools_->deleteAllMarkers();

//  ROS_INFO_STREAM("publishing path [" << i << "]...");

  assert(0 <= i && i < visual_path_array_.size());

  if(0 != i)
  {
    for(int j=0; j < i; j++)
    {
      visual_tools_->publishCylinder(visual_path_array_[j].start_pt,
                                     visual_path_array_[j].end_pt,
                                     rviz_visual_tools::GREY, visual_path_array_[i].diameter, "Cylinder");
    }
  }

  visual_tools_->publishCylinder(visual_path_array_[i].start_pt,
                                 visual_path_array_[i].end_pt,
                                 rviz_visual_tools::TRANSLUCENT, visual_path_array_[i].diameter, "Cylinder");

  visual_tools_->trigger();
}

void framefab_visual_tools::FrameFabVisualTool::cleanUpAllPaths()
{
  visual_tools_->deleteAllMarkers();
  visual_tools_->trigger();
}