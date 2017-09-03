//
// Created by yijiangh on 6/27/17.
//

#include <framefab_core/visual_tools/framefab_visual_tool.h>

#include <math.h>

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

    // fill in element info
    tf::pointMsgToEigen(path_array[i].start_pt, v_unit_path.start_pt);
    tf::pointMsgToEigen(path_array[i].end_pt, v_unit_path.end_pt);
    v_unit_path.type =
        static_cast<framefab_visual_tools::UNIT_PATH_TYPE>(path_array[i].type);
    v_unit_path.diameter = path_array[i].element_diameter;

    // fill in feasible orientations
    v_unit_path.oriented_st_pts.clear();
    Eigen::Vector3d avr_vec = Eigen::Vector3d(0,0,0);
    int m = path_array[i].feasible_orients.size();
    for(int j=0; j < m; j++)
    {
      Eigen::Vector3d e;
      tf::vectorMsgToEigen(path_array[i].feasible_orients[j], e);

      e = e * 0.001 * 15;
      avr_vec  = avr_vec  + e;
      e = e + v_unit_path.start_pt;

      v_unit_path.oriented_st_pts.push_back(e);
    }
    avr_vec = avr_vec / avr_vec.norm() * 0.001 * 15;
    avr_vec = avr_vec + v_unit_path.start_pt;

    v_unit_path.avr_orient_vec = avr_vec;

    visual_path_array.push_back(v_unit_path);
  }
}

void framefab_visual_tools::FrameFabVisualTool::visualizeAllPaths()
{
  visual_tools_->deleteAllMarkers();

//  ROS_INFO_STREAM("publishing all paths...");

  for(std::size_t i = 0; i < visual_path_array_.size(); i++)
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

    visual_tools_->publishArrow(
        visual_tools_->getVectorBetweenPoints(visual_path_array_[i].start_pt,
                                              visual_path_array_[i].end_pt),
        type_color, rviz_visual_tools::XXXXSMALL,
        (visual_path_array_[i].start_pt - visual_path_array_[i].end_pt).norm());
  }

  visual_tools_->trigger();
}

void framefab_visual_tools::FrameFabVisualTool::visualizePathUntil(int i)
{
  visual_tools_->deleteAllMarkers();

//  ROS_INFO_STREAM("publishing path [" << i << "]...");

  assert(0 <= i && i < visual_path_array_.size());

  if(0 != i)
  {
    for(std::size_t j = 0; j < i; j++)
    {
      visual_tools_->publishArrow(
          visual_tools_->getVectorBetweenPoints(visual_path_array_[j].start_pt,
                                                visual_path_array_[j].end_pt),
          rviz_visual_tools::GREY, rviz_visual_tools::XXXXSMALL,
          (visual_path_array_[j].start_pt - visual_path_array_[j].end_pt).norm());
    }
  }

  visual_tools_->publishArrow(
      visual_tools_->getVectorBetweenPoints(visual_path_array_[i].start_pt,
                                            visual_path_array_[i].end_pt),
      rviz_visual_tools::CYAN, rviz_visual_tools::XXXXSMALL,
      (visual_path_array_[i].start_pt - visual_path_array_[i].end_pt).norm());

  visual_tools_->trigger();
}

void framefab_visual_tools::FrameFabVisualTool::cleanUpAllPaths()
{
  visual_tools_->deleteAllMarkers();
  visual_tools_->trigger();
}

void framefab_visual_tools::FrameFabVisualTool::visualizeFeasibleOrientations(int i, bool solid)
{
  assert(0 <= i && i < visual_path_array_.size());

  for(int j = 0; j < visual_path_array_[i].oriented_st_pts.size(); j++)
  {
    rviz_visual_tools::colors type_color;
    if(!solid)
    {
      type_color = rviz_visual_tools::TRANSLUCENT;
    }
    else
    {
      type_color = rviz_visual_tools::GREEN;
    }

    visual_tools_->publishCylinder(
        visual_path_array_[i].start_pt,
        visual_path_array_[i].oriented_st_pts[j],
        type_color,
        0.0003,
        "orientation_cylinder");
  }

  rviz_visual_tools::colors type_color_avr;
  if(!solid)
  {
    type_color_avr = rviz_visual_tools::TRANSLUCENT;
  }
  else
  {
    type_color_avr = rviz_visual_tools::PURPLE;
  }

  visual_tools_->publishCylinder(
      visual_path_array_[i].start_pt,
      visual_path_array_[i].avr_orient_vec,
      type_color_avr,
      0.0003,
      "orientation_cylinder");

  visual_tools_->trigger();
}