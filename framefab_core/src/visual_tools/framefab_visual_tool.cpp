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
    tf::pointMsgToEigen(path_array[i].shrinked_start_pt, v_unit_path.start_pt);
    tf::pointMsgToEigen(path_array[i].shrinked_end_pt, v_unit_path.end_pt);
    v_unit_path.type =
        static_cast<framefab_visual_tools::UNIT_PATH_TYPE>(path_array[i].type);
    v_unit_path.diameter = path_array[i].element_diameter;

    // fill in feasible orientations
    v_unit_path.feasible_orientations.clear();
    Eigen::Vector3d avr_vec(0,0,0);
    for(int j=0; j < path_array[i].feasible_orients.size(); j++)
    {
      Eigen::Affine3d orient = Eigen::Affine3d::Identity();
      Eigen::Vector3d eigen_orient;
      tf::vectorMsgToEigen(path_array[i].feasible_orients[j], eigen_orient);

      orient.translation().x() = v_unit_path.start_pt.x();
      orient.translation().y() = v_unit_path.start_pt.y();
      orient.translation().z() = v_unit_path.start_pt.z();

      orient = orient * Eigen::AngleAxisd(
          acos(Eigen::Vector3d::UnitX().dot(eigen_orient)),
          Eigen::Vector3d::UnitX().cross(eigen_orient));

      v_unit_path.feasible_orientations.push_back(orient);

      avr_vec = avr_vec + eigen_orient;
    }

    avr_vec = avr_vec / path_array[i].feasible_orients.size();

    Eigen::Affine3d avr_orient = Eigen::Affine3d::Identity();
    avr_orient.translation().x() = v_unit_path.start_pt.x();
    avr_orient.translation().y() = v_unit_path.start_pt.y();
    avr_orient.translation().z() = v_unit_path.start_pt.z();

    avr_orient = avr_orient * Eigen::AngleAxisd(
        acos(Eigen::Vector3d::UnitX().dot(avr_vec)),
        Eigen::Vector3d::UnitX().cross(avr_vec));

    v_unit_path.average_f_orient = avr_orient;

    // alternative orientation marker
    v_unit_path.oriented_st_pts.clear();
    v_unit_path.average_orient_vec = Eigen::Vector3d(0,0,0);
    for(int j=0; j < path_array[i].feasible_orients.size(); j++)
    {
      Eigen::Vector3d e;
      tf::vectorMsgToEigen(path_array[i].feasible_orients[j], e);

      e = e * 0.001 * 15;
      e = e + v_unit_path.start_pt;

      v_unit_path.oriented_st_pts.push_back(e);

      v_unit_path.average_orient_vec  = v_unit_path.average_orient_vec  + e;
    }
    v_unit_path.average_orient_vec =
        v_unit_path.average_orient_vec /  v_unit_path.oriented_st_pts.size();

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

  for(int j=0; j < visual_path_array_[i].oriented_st_pts.size(); j++)
  {
    visual_tools_->publishLine(
        visual_path_array_[i].start_pt,
        visual_path_array_[i].oriented_st_pts[j],
        rviz_visual_tools::GREEN,
        0.001);
  }

  visual_tools_->publishLine(
      visual_path_array_[i].start_pt,
      visual_path_array_[i].average_orient_vec,
      rviz_visual_tools::MAGENTA,
      0.001);
//  visualizeFeasibleOrientations(i);

  visual_tools_->trigger();
}

void framefab_visual_tools::FrameFabVisualTool::cleanUpAllPaths()
{
  visual_tools_->deleteAllMarkers();
  visual_tools_->trigger();
}

void framefab_visual_tools::FrameFabVisualTool::visualizeFeasibleOrientations(int i)
{
  assert(0 <= i && i < visual_path_array_.size());

  for(int j=0; j < visual_path_array_[i].feasible_orientations.size(); j++)
  {
    visual_tools_->publishArrow(
        visual_path_array_[i].feasible_orientations[j],
        rviz_visual_tools::GREEN, rviz_visual_tools::XXXSMALL);
  }

  visual_tools_->publishArrow(
      visual_path_array_[i].average_f_orient,
      rviz_visual_tools::MAGENTA, rviz_visual_tools::XXXSMALL);
}