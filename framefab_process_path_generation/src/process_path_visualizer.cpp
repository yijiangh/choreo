//
// Created by yijiangh on 6/5/17.
//

#include <framefab_process_path_generation/process_path_visualizer.h>

namespace framefab_process_path
{
//Function for constructing quaternion starting from Euler rotations XYZ
Eigen::Quaternion<double> ProcessPathVisualizer::eulerToQuat(double rotX, double rotY, double rotZ)
{
    Eigen::Matrix3d m;
    m = Eigen::AngleAxisd(rotX, Eigen::Vector3d::UnitX())
        * Eigen::AngleAxisd(rotY, Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(rotZ, Eigen::Vector3d::UnitZ());

    Eigen::AngleAxis<double> aa;
    aa = Eigen::AngleAxisd(m);

    Eigen::Quaternion<double> quat;
    quat = Eigen::Quaternion<double>(aa);
    return quat;
}

visualization_msgs::Marker ProcessPathVisualizer::createMarker(double transX, double transY, double transZ, double rotX, double rotY, double rotZ,
                                        descartes_trajectory::AxialSymmetricPt::FreeAxis axis =
                                        descartes_trajectory::AxialSymmetricPt::X_AXIS)
{
  static int count;
  visualization_msgs::Marker marker;
  marker.header.frame_id = "base_link";
  marker.header.stamp = ros::Time();
  marker.ns = "my_namespace";
  marker.id = count;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;
  marker.lifetime = ros::Duration(0);

  marker.pose.position.x = transX;
  marker.pose.position.y = transY;
  marker.pose.position.z = transZ;

  //To calculate the quaternion values we first define an AngleAxis object using Euler rotations, then convert it
  Eigen::Quaternion<double> quat;
  if(axis == descartes_trajectory::AxialSymmetricPt::X_AXIS)
  {
    quat = eulerToQuat(rotX, rotY, rotZ);
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
  } else if(axis == descartes_trajectory::AxialSymmetricPt::Y_AXIS)
  {
    quat = eulerToQuat(rotX, rotY, rotZ) * eulerToQuat(0, 0, M_PI / 2);
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
  } else if(axis == descartes_trajectory::AxialSymmetricPt::Z_AXIS)
  {
    quat = eulerToQuat(rotX, rotY, rotZ) * eulerToQuat(0, 3 * (M_PI / 2), 0);
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
  }

  marker.pose.orientation.x = quat.x();
  marker.pose.orientation.y = quat.y();
  marker.pose.orientation.z = quat.z();
  marker.pose.orientation.w = quat.w();
  marker.scale.x = 0.02;
  marker.scale.y = 0.002;
  marker.scale.z = 0.002;
  marker.color.a = 1.0;	//Alpha

  count++;
  return marker;
}

void ProcessPathVisualizer::createVisualFrame(Eigen::Affine3d pose, std::vector<visualization_msgs::Marker> & markervec)
{
  Eigen::Vector3d translations;
  translations = pose.translation();
  Eigen::Vector3d rotationsXYZ;
  rotationsXYZ = pose.rotation().eulerAngles(0,1,2);

  markervec.push_back(createMarker(translations[0], translations[1], translations[2],
                                   rotationsXYZ[0], rotationsXYZ[1], rotationsXYZ[2], descartes_trajectory::AxialSymmetricPt::X_AXIS));
  markervec.push_back(createMarker(translations[0], translations[1], translations[2],
                                   rotationsXYZ[0], rotationsXYZ[1], rotationsXYZ[2], descartes_trajectory::AxialSymmetricPt::Y_AXIS));
  markervec.push_back(createMarker(translations[0], translations[1], translations[2],
                                   rotationsXYZ[0], rotationsXYZ[1], rotationsXYZ[2], descartes_trajectory::AxialSymmetricPt::Z_AXIS));
}

visualization_msgs::MarkerArray ProcessPathVisualizer::createMarkerArray(std::vector<Eigen::Affine3d> poses)
{
  std::vector<visualization_msgs::Marker> markerVector;
  int vectorSize;
  vectorSize = poses.size();

  //Generate markers for every pose, put them in markerVector
  for(int i = 0; i < vectorSize; i++)
  {
    createVisualFrame(poses[i], markerVector);
  }

  //Copy vector to array so we can return it as a MarkerArray type.
  int size = markerVector.size();
  visualization_msgs::MarkerArray ma;
  ma.markers.resize(size);
  for(int i = 0;i < size;i++)
  {
    ma.markers[i] = markerVector[i];
  }
  return ma;
}

}// namespace framefab_process_path
