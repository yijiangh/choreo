#include <choreo_visual_tools/choreo_visual_tools.h>

#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

#include <boost/filesystem.hpp>

const static rviz_visual_tools::colors PICK_COLOR = rviz_visual_tools::BLUE;
const static rviz_visual_tools::colors PICK_NGH_COLOR = rviz_visual_tools::TRANSLUCENT_DARK;
const static rviz_visual_tools::colors PLACE_COLOR = rviz_visual_tools::GREEN;
const static rviz_visual_tools::colors PLACE_NGH_COLOR = rviz_visual_tools::TRANSLUCENT_DARK;

const static rviz_visual_tools::colors SUPPORT_SURFACE_COLOR = rviz_visual_tools::GREY;

const static rviz_visual_tools::colors EXIST_ELEMENT_COLOR = rviz_visual_tools::DARK_GREY ;

const static std::string MESH_FILE_PREFIX = "file://";

// TODO: mesh scale should read from input model param
// default millimeter
const static double PNP_MESH_SCALE = 0.001;

void choreo_visual_tools::ChoreoVisualTools::init(std::string frame_name, std::string marker_topic)
{
  visual_tools_.reset(
      new rviz_visual_tools::RvizVisualTools(frame_name, marker_topic));

  // Clear messages
  visual_tools_->deleteAllMarkers();
  visual_tools_->enableBatchPublishing();
}

void choreo_visual_tools::ChoreoVisualTools::convertPathVisual(
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
        static_cast<choreo_visual_tools::UNIT_PATH_TYPE>(path_array[i].type);
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

void choreo_visual_tools::ChoreoVisualTools::convertWireFrameVisual(
    const PathArray& path_array, VisualPathArray& visual_path_array)
{
  visual_path_array.clear();

  for(int i=0; i < path_array_.size(); i++)
  {
    VisualUnitProcessPath v_unit_path;

    // fill in element info
    tf::pointMsgToEigen(path_array[i].start_pt, v_unit_path.start_pt);
    tf::pointMsgToEigen(path_array[i].end_pt, v_unit_path.end_pt);
    v_unit_path.layer_id = path_array[i].layer_id;
    v_unit_path.diameter = path_array[i].element_diameter;

    visual_path_array.push_back(v_unit_path);
  }
}

void choreo_visual_tools::ChoreoVisualTools::visualizeAllPaths()
{
  visual_tools_->deleteAllMarkers();

//  ROS_INFO_STREAM("publishing all paths...");

  for(std::size_t i = 0; i < visual_path_array_.size(); i++)
  {
    rviz_visual_tools::colors type_color;

    if(choreo_visual_tools::UNIT_PATH_TYPE::SUPPORT ==  visual_path_array_[i].type)
    {
      type_color = rviz_visual_tools::BLUE;
    }

    if(choreo_visual_tools::UNIT_PATH_TYPE::CREATE ==  visual_path_array_[i].type)
    {
      type_color = rviz_visual_tools::YELLOW;
    }

    if(choreo_visual_tools::UNIT_PATH_TYPE::CONNECT ==  visual_path_array_[i].type)
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

void choreo_visual_tools::ChoreoVisualTools::visualizeAllSequencePickNPlace()
{
  visual_tools_->deleteAllMarkers();

  assert(as_pnp_.sequenced_elements.size() > 0);

  Eigen::Affine3d zero_pose = Eigen::Affine3d::Identity();

  for(std::size_t i = 0; i < as_pnp_.sequenced_elements.size(); i++)
  {
    const auto& current_as = as_pnp_.sequenced_elements[i];

    assert(boost::filesystem::exists(current_as.file_path + current_as.pick_element_geometry_file_name));
    assert(boost::filesystem::exists(current_as.file_path + current_as.place_element_geometry_file_name));

    visual_tools_->publishMesh(
        zero_pose,
        MESH_FILE_PREFIX + current_as.file_path + current_as.pick_element_geometry_file_name,
        PICK_COLOR,
        PNP_MESH_SCALE);

    visual_tools_->publishMesh(
        zero_pose,
        MESH_FILE_PREFIX + current_as.file_path + current_as.place_element_geometry_file_name,
        PLACE_COLOR,
        PNP_MESH_SCALE);
  }

  visual_tools_->trigger();
}

void choreo_visual_tools::ChoreoVisualTools::visualizePathUntil(int i)
{
  visual_tools_->deleteAllMarkers();

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

void choreo_visual_tools::ChoreoVisualTools::visualizeSequencePickNPlaceUntil(int i)
{
  visual_tools_->deleteAllMarkers();

  assert(as_pnp_.sequenced_elements.size() > 0 && i < as_pnp_.sequenced_elements.size() && i >= 0);

  Eigen::Affine3d zero_pose = Eigen::Affine3d::Identity();

  const auto& current_as = as_pnp_.sequenced_elements[i];

  assert(boost::filesystem::exists(current_as.file_path + current_as.pick_element_geometry_file_name));
  assert(boost::filesystem::exists(current_as.file_path + current_as.place_element_geometry_file_name));

  visual_tools_->publishMesh(
      zero_pose,
      MESH_FILE_PREFIX + current_as.file_path + current_as.pick_element_geometry_file_name,
      PICK_COLOR,
      PNP_MESH_SCALE);

  visual_tools_->publishMesh(
      zero_pose,
      MESH_FILE_PREFIX + current_as.file_path + current_as.place_element_geometry_file_name,
      PLACE_COLOR,
      PNP_MESH_SCALE);

  // TODO: visualize supporting surfaces


  if(0 != i)
  {
    // visualize everything that are placed already
    for(std::size_t j = 0; j < i; j++)
    {
      const auto& as = as_pnp_.sequenced_elements[j];
      rviz_visual_tools::colors p_c;

      assert(boost::filesystem::exists(as.file_path + as.place_element_geometry_file_name));

      if (std::find(current_as.place_contact_ngh_ids.begin(),current_as.place_contact_ngh_ids.end(),j)
          != current_as.place_contact_ngh_ids.end())
      {
        p_c = PLACE_NGH_COLOR;
      }
      else
      {
        p_c = EXIST_ELEMENT_COLOR;
      }

      visual_tools_->publishMesh(
          zero_pose,
          MESH_FILE_PREFIX + as.file_path + as.place_element_geometry_file_name,
          p_c,
          PNP_MESH_SCALE);
    }

    // everything still in the picking pile
    for(std::size_t j = i; j < as_pnp_.sequenced_elements.size(); j++)
    {
      const auto& as = as_pnp_.sequenced_elements[j];
      rviz_visual_tools::colors p_c;

      assert(boost::filesystem::exists(as.file_path + as.pick_element_geometry_file_name));

      if (std::find(current_as.pick_contact_ngh_ids.begin(),current_as.pick_contact_ngh_ids.end(),j)
          != current_as.pick_contact_ngh_ids.end())
      {
        p_c = PICK_NGH_COLOR;
      }
      else
      {
        p_c = EXIST_ELEMENT_COLOR;
      }

      visual_tools_->publishMesh(
          zero_pose,
          MESH_FILE_PREFIX + as.file_path + as.pick_element_geometry_file_name,
          p_c,
          PNP_MESH_SCALE);
    }
  }

  visual_tools_->trigger();
}

void choreo_visual_tools::ChoreoVisualTools::visualizeFeasibleOrientations(int i, bool solid)
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

void visualizeGraspPickNPlace(int index, int grasp_id, bool visualize_ee)
{

}

void choreo_visual_tools::ChoreoVisualTools::visualizeAllWireFrame()
{
  visual_tools_->deleteAllMarkers();

  for(std::size_t i = 0; i < visual_path_array_.size(); i++)
  {
    rviz_visual_tools::colors type_color =
        static_cast<rviz_visual_tools::colors>((visual_path_array_[i].layer_id) % 14 + 1);

    visual_tools_->publishCylinder(visual_path_array_[i].start_pt,
                                   visual_path_array_[i].end_pt,
                                   type_color,
                                   visual_path_array_[i].diameter);
  }

  visual_tools_->trigger();
}

void choreo_visual_tools::ChoreoVisualTools::visualizeSupportSurfaces()
{
  assert(boost::filesystem::exists(as_pnp_.file_path + as_pnp_.pick_support_surface_file_names));
  assert(boost::filesystem::exists(as_pnp_.file_path + as_pnp_.place_support_surface_file_names));

  visual_tools_->publishMesh(
      zero_pose,
      MESH_FILE_PREFIX + as_pnp_.file_path + as_pnp_.pick_support_surface_file_names,
      SUPPORT_SURFACE_COLOR,
      PNP_MESH_SCALE);

//  visual_tools_->publishMesh(
//      zero_pose,
//      MESH_FILE_PREFIX + current_as.file_path + as_pnp_.place   _support_surface_file_names,
//      SUPPORT_SURFACE_COLOR,
//      PNP_MESH_SCALE);
}

void choreo_visual_tools::ChoreoVisualTools::cleanUpAllPaths()
{
  visual_tools_->deleteAllMarkers();
  visual_tools_->trigger();
}
