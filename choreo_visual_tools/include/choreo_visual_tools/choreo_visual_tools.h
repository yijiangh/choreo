#ifndef CHOREO_VISUAL_TOOL_H
#define CHOREO_VISUAL_TOOL_H

// C++
#include <string>
#include <vector>

// ROS
#include <ros/ros.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

// For visualizing things in rviz
#include <rviz_visual_tools/rviz_visual_tools.h>

#include <framefab_msgs/ElementCandidatePoses.h>
#include <framefab_msgs/AssemblySequencePickNPlace.h>

namespace choreo_visual_tools
{

// TODO: not used in picknplace
enum UNIT_PATH_TYPE
{
  SUPPORT=2,
  CREATE=1,
  CONNECT=0
};

struct VisualUnitProcessPath
{
  Eigen::Vector3d start_pt;
  Eigen::Vector3d end_pt;

  std::vector<Eigen::Vector3d> oriented_st_pts;
  Eigen::Vector3d avr_orient_vec;

  UNIT_PATH_TYPE type;
  double diameter;
  int layer_id;
};

class ChoreoVisualTools
{
  typedef std::vector<framefab_msgs::ElementCandidatePoses> PathArray;
  typedef std::vector<choreo_visual_tools::VisualUnitProcessPath> VisualPathArray;

 public:
  ChoreoVisualTools(){}
  virtual ~ChoreoVisualTools(){}

  void init(std::string frame_name, std::string marker_topic);

  // set visual wire frames
  // visualize layer decomposition and wire frames
  // when no assembly seq (with grasps) is inputted
  void setVisualWireFrame(const PathArray& path_array)
  {
    path_array_ = path_array;
    convertWireFrameVisual(path_array_, visual_path_array_);
  }

  // set visual assembly seq for spatial extrusion
  void setProcessPath(const PathArray& path_array)
  {
    path_array_ = path_array;
    convertPathVisual(path_array_, visual_path_array_);
  }

  int getPathArraySize() { return visual_path_array_.size(); }

  // VISUALIZE SEQUENCED EXTRUSIONS
  //
  // visualize all extrusion sequence at once, mark different types
  // of process: blue: support, yellow: create, red: connect
  void visualizeAllPaths();

  // visualize extrusion sequence until the given index
  // all elements are painted grey except element No. index
  void visualizePathUntil(int index);

  // visualize feasible orientations using green lines
  void visualizeFeasibleOrientations(int index, bool solid);
  //
  // VISUALIZE SEQUENCED EXTRUSIONS END

  //
  void visualizeAllWireFrame();

  //TODO: not implemented now
  void visualizeWireFrameUntilLayer(int index);

  // PICKNPLACE
  //
  void setAssemblySequencePickNPlace(const framefab_msgs::AssemblySequencePickNPlace& as_pnp) { as_pnp_ = as_pnp; }

  void visualizeSequencePickNPlaceUntil(int index);

  void visualizeGraspPickNPlace(int index, int grasp_id, bool visualize_ee);

  void visualizeAllSequencePickNPlace();

  void visualizeSupportSurfaces();
  //
  // PICKNPLACE END

  void cleanUpAllPaths();

 protected:
  void convertPathVisual(const PathArray& path_array, VisualPathArray& visual_path_array);
  void convertWireFrameVisual(const PathArray& path_array, VisualPathArray& visual_path_array);

 private:
  ros::NodeHandle nh_;

  rviz_visual_tools::RvizVisualToolsPtr visual_tools_;

  PathArray path_array_;
  VisualPathArray visual_path_array_;

  // TODO: should try to make two assembly type's function unites under one scheme
  framefab_msgs::AssemblySequencePickNPlace as_pnp_;
};
}

#endif
