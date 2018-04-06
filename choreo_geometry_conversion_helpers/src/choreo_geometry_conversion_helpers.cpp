#include <choreo_geometry_conversion_helpers/choreo_geometry_conversion_helpers.h>

// mesh collision geometry import
#include <eigen_conversions/eigen_msg.h>
#include <geometric_shapes/shape_operations.h>

namespace{
void planeAxesToEigenMatrixImpl(const Eigen::Vector3d& x_axis, const Eigen::Vector3d& y_axis, const Eigen::Vector3d& z_axis,
                            Eigen::Matrix3d& m)
{
  m = Eigen::Matrix3d::Identity();
  m.col(0) = x_axis;
  m.col(1) = y_axis;
  m.col(2) = z_axis;
}

void planeToEigenAffine3dImpl(const Eigen::Vector3d& o, const Eigen::Matrix3d& m, Eigen::Affine3d& e)
{
  e = Eigen::Affine3d::Identity();
  e.matrix().block<3,3>(0,0) = m;
  e.matrix().col(3).head<3>() = o;
}

} // anon util namespace

namespace choreo_geometry_conversion_helpers
{

void planeAxesToEigenMatrix(const Eigen::Vector3d& x_axis, const Eigen::Vector3d& y_axis, const Eigen::Vector3d& z_axis,
                            Eigen::Matrix3d& m)
{
  planeAxesToEigenMatrixImpl(x_axis, y_axis, z_axis, m);
}

void planeAxesToQuaternionMsg(const Eigen::Vector3d& x_axis, const Eigen::Vector3d& y_axis, const Eigen::Vector3d& z_axis,
                              geometry_msgs::Quaternion& q_msg)
{
  Eigen::Matrix3d m;
  planeAxesToEigenMatrixImpl(x_axis, y_axis, z_axis, m);

  Eigen::Quaterniond e_q(m);
  tf::quaternionEigenToMsg(e_q, q_msg);
}

void planeToEigenAffine3d(const Eigen::Vector3d origin,
                          const Eigen::Vector3d& x_axis, const Eigen::Vector3d& y_axis, const Eigen::Vector3d& z_axis,
                          Eigen::Affine3d& e)
{
  Eigen::Matrix3d m;
  planeAxesToEigenMatrixImpl(x_axis, y_axis, z_axis, m);

  planeToEigenAffine3dImpl(origin, m, e);
}

void planeToEigenAffine3d(const Eigen::Vector3d& origin, const Eigen::Matrix3d& orientation, Eigen::Affine3d& e)
{
  planeToEigenAffine3dImpl(origin, orientation, e);
}

void planeToPoseMsg(const Eigen::Vector3d origin,
                    const Eigen::Vector3d& x_axis, const Eigen::Vector3d& y_axis, const Eigen::Vector3d& z_axis,
                    geometry_msgs::Pose& p)
{
  Eigen::Affine3d e;

  Eigen::Matrix3d m;
  planeAxesToEigenMatrixImpl(x_axis, y_axis, z_axis, m);

  planeToEigenAffine3dImpl(origin, m, e);
  tf::poseEigenToMsg(e, p);
}

void planeToPoseMsg(const Eigen::Vector3d& origin, const Eigen::Matrix3d& orientation,
                    geometry_msgs::Pose& p)
{
  Eigen::Affine3d e;
  planeToEigenAffine3dImpl(origin, orientation, e);
  tf::poseEigenToMsg(e, p);
}

void savedSTLToMeshShapeMsg(const std::string& file_path, const Eigen::Vector3d& scale_vector, shape_msgs::Mesh& mesh)
{
  shapes::Mesh* m = shapes::createMeshFromResource(file_path, scale_vector);

  shapes::ShapeMsg mesh_msg;
  shapes::constructMsgFromShape(m, mesh_msg);

  mesh = boost::get<shape_msgs::Mesh>(mesh_msg);
}

}

