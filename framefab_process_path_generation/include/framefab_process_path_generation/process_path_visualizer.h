//
// Created by yijiangh on 6/5/17.
//

#ifndef FRAMEFAB_PATH_GENERATION_PROCESS_PATH_VISUALIZATION_H
#define FRAMEFAB_PATH_GENERATION_PROCESS_PATH_VISUALIZATION_H

//Include visualization markers for RViz
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

//Include Eigen geometry header for rotations
#include <Eigen/Geometry>

// Includes the descartes trajectory type we will be using
#include <descartes_trajectory/axial_symmetric_pt.h>

namespace framefab_process_path
{

class ProcessPathVisualizer
{
 public:
  ProcessPathVisualizer() {};
  ~ProcessPathVisualizer() {};

 public:
  Eigen::Quaternion<double> eulerToQuat(double rotX, double rotY, double rotZ);

  visualization_msgs::Marker createMarker(double transX,
                                          double transY,
                                          double transZ,
                                          double rotX,
                                          double rotY,
                                          double rotZ,
                                          descartes_trajectory::AxialSymmetricPt::FreeAxis axis);

  void createVisualFrame(Eigen::Affine3d pose, std::vector <visualization_msgs::Marker> &markervec);

  visualization_msgs::MarkerArray createMarkerArray(std::vector <Eigen::Affine3d> poses);

};

}// namespace framefab_process_path

#endif //FRAMEFAB_PATH_GENERATION_PROCESS_PATH_VISUALIZATION_H
