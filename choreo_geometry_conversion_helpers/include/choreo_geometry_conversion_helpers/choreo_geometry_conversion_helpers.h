#ifndef CHOREO_GEOMETRY_CONVERSION_HELPERS
#define CHOREO_GEOMETRY_CONVERSION_HELPERS

#include "Eigen/Core"
#include "Eigen/Geometry"

// msgs
#include <shape_msgs/Mesh.h>

namespace choreo_geometry_conversion_helpers
{

void planeAxesToEigenMatrix(const Eigen::Vector3d& x_axis, const Eigen::Vector3d& y_axis, const Eigen::Vector3d& z_axis,
                            Eigen::Matrix3d& m);

void planeToEigenAffine3d(const Eigen::Vector3d origin,
                          const Eigen::Vector3d& x_axis, const Eigen::Vector3d& y_axis, const Eigen::Vector3d& z_axis,
                          Eigen::Affine3d& e);

void planeToEigenAffine3d(const Eigen::Vector3d& origin, const Eigen::Matrix3d& orientation,
                          Eigen::Affine3d& e);

void savedSTLToMeshShapeMsg(const std::string& file_path, const Eigen::Vector3d& scale_vector,
                            shape_msgs::Mesh& mesh);

}

#endif
