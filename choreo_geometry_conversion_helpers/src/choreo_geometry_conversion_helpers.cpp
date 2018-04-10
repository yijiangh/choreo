#include <choreo_geometry_conversion_helpers/choreo_geometry_conversion_helpers.h>

// mesh collision geometry import
#include <eigen_conversions/eigen_msg.h>
#include <geometric_shapes/shape_operations.h>

#include <boost/filesystem.hpp>

namespace{

void convertOrientationVectors(
    const std::vector<geometry_msgs::Vector3>& orients_msg,
    std::vector<Eigen::Matrix3d>& m_orients)
{
  m_orients.clear();

  for(auto v : orients_msg)
  {
    // eigen_vec = local z axis
    Eigen::Vector3d eigen_vec;
    tf::vectorMsgToEigen(v, eigen_vec);

    // TODO: this should be removed
    eigen_vec *= -1.0;
    eigen_vec.normalize();

    // construct local x axis & y axis
    Eigen::Vector3d candidate_dir = Eigen::Vector3d::UnitX();
    if ( std::abs(eigen_vec.dot(Eigen::Vector3d::UnitX())) > 0.8 )
    {
      // if z axis = UnitX,
      candidate_dir = Eigen::Vector3d::UnitY();
    }

    Eigen::Vector3d y_vec = eigen_vec.cross(candidate_dir).normalized();

    Eigen::Vector3d x_vec = y_vec.cross(eigen_vec).normalized();

    // JM
    Eigen::Matrix3d m = Eigen::Matrix3d::Identity();
    m.col(0) = x_vec;
    m.col(1) = y_vec;
    m.col(2) = eigen_vec;

    m_orients.push_back(m);
  }
}

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
const static std::string FILE_URL_PREFIX = "file://";

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

void planeToEigenAffine3d(const Eigen::Vector3d& origin,
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

void planeToPoseMsg(const Eigen::Vector3d& origin,
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
  assert(boost::filesystem::exists(file_path));

  shapes::Mesh* m = shapes::createMeshFromResource(FILE_URL_PREFIX + file_path, scale_vector);

  shapes::ShapeMsg mesh_msg;
  shapes::constructMsgFromShape(m, mesh_msg);

  mesh = boost::get<shape_msgs::Mesh>(mesh_msg);
}

moveit_msgs::CollisionObject savedSTLToCollisionObjectMsg(const std::string& file_path,
                                                          const Eigen::Vector3d& scale_vector,
                                                          const std::string& frame_id,
                                                          const geometry_msgs::Pose& p,
                                                          int object_operation)
{
  moveit_msgs::CollisionObject co;

  savedSTLToCollisionObjectMsg(file_path, scale_vector, frame_id, p, co, object_operation);

  return co;
}

void savedSTLToCollisionObjectMsg(const std::string& file_path,
                                  const Eigen::Vector3d& scale_vector,
                                  const std::string& frame_id,
                                  const geometry_msgs::Pose& p,
                                  moveit_msgs::CollisionObject& co,
                                  int object_operation)
{
  assert(boost::filesystem::exists(file_path));

  boost::filesystem::path boost_path(file_path);

  savedSTLToCollisionObjectMsg(file_path, scale_vector, frame_id,
                               boost_path.filename().string(),
                               p, co, object_operation);
}

void savedSTLToCollisionObjectMsg(const std::string& file_path,
                                  const Eigen::Vector3d& scale_vector,
                                  const std::string& frame_id,
                                  const std::string& obj_id,
                                  const geometry_msgs::Pose& p,
                                  moveit_msgs::CollisionObject& co,
                                  int op)
{
  assert(boost::filesystem::exists(file_path));

  co.header.frame_id = frame_id;
  co.id = obj_id;

  co.meshes.resize(1);
  co.mesh_poses.resize(1);

  savedSTLToMeshShapeMsg(file_path, scale_vector, co.meshes[0]);
  co.mesh_poses[0] = p;

  assert(co.ADD == op || co.REMOVE == op || co.APPEND == op || co.MOVE == op);
  co.operation = op;
}

moveit_msgs::AttachedCollisionObject savedSTLToAttachedCollisionObjectMsg(
    const std::string& file_path,
    const Eigen::Vector3d& scale_vector,
    const std::string& frame_id,
    const std::string& link_id,
    const geometry_msgs::Pose& p,
    int object_operation)
{
  moveit_msgs::AttachedCollisionObject ato;

  savedSTLToAttachedCollisionObjectMsg(file_path, scale_vector, frame_id, link_id, p, ato, object_operation);

  return ato;
}

void savedSTLToAttachedCollisionObjectMsg(const std::string& file_path,
                                          const Eigen::Vector3d& scale_vector,
                                          const std::string& frame_id,
                                          const std::string& link_id,
                                          const geometry_msgs::Pose& p,
                                          moveit_msgs::AttachedCollisionObject& aco,
                                          int object_operation)
{
  assert(boost::filesystem::exists(file_path));

  boost::filesystem::path boost_path(file_path);

  savedSTLToAttachedCollisionObjectMsg(file_path, scale_vector, frame_id, link_id,
                                       boost_path.filename().string(),
                                       p, aco, object_operation);
}

void savedSTLToAttachedCollisionObjectMsg(const std::string& file_path,
                                          const Eigen::Vector3d& scale_vector,
                                          const std::string& frame_id,
                                          const std::string& link_id,
                                          const std::string& obj_id,
                                          const geometry_msgs::Pose& p,
                                          moveit_msgs::AttachedCollisionObject& aco,
                                          int object_operation)
{
  aco.link_name = link_id;

  // use default touch link

  savedSTLToCollisionObjectMsg(file_path, scale_vector, frame_id, obj_id, p, aco.object, object_operation);
}

}