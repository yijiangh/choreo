#ifndef CHOREO_GEOMETRY_CONVERSION_HELPERS
#define CHOREO_GEOMETRY_CONVERSION_HELPERS

#include "Eigen/Core"
#include "Eigen/Geometry"

// msgs
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <shape_msgs/Mesh.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/AttachedCollisionObject.h>

namespace choreo_geometry_conversion_helpers
{

// Rotation representations
void planeAxesToEigenMatrix(const Eigen::Vector3d& x_axis, const Eigen::Vector3d& y_axis, const Eigen::Vector3d& z_axis,
                            Eigen::Matrix3d& m);

void planeAxesToQuaternionMsg(const Eigen::Vector3d& x_axis, const Eigen::Vector3d& y_axis, const Eigen::Vector3d& z_axis,
                            geometry_msgs::Quaternion& q_msg);

// Pose (position + rotation) representations
void planeToEigenAffine3d(const Eigen::Vector3d& origin,
                          const Eigen::Vector3d& x_axis, const Eigen::Vector3d& y_axis, const Eigen::Vector3d& z_axis,
                          Eigen::Affine3d& e);

void planeToEigenAffine3d(const Eigen::Vector3d& origin, const Eigen::Matrix3d& orientation,
                          Eigen::Affine3d& e);

void planeToPoseMsg(const Eigen::Vector3d& origin,
                    const Eigen::Vector3d& x_axis, const Eigen::Vector3d& y_axis, const Eigen::Vector3d& z_axis,
                    geometry_msgs::Pose& p);

void planeToPoseMsg(const Eigen::Vector3d& origin, const Eigen::Matrix3d& orientation,
                    geometry_msgs::Pose& p);

// import geometry related utils
void savedSTLToMeshShapeMsg(const std::string& file_path, const Eigen::Vector3d& scale_vector,
                            shape_msgs::Mesh& mesh);

// file path is full path with filename
void savedSTLToCollisionObjectMsg(const std::string& file_path,
                                  const Eigen::Vector3d& scale_vector,
                                  const std::string& frame_id,
                                  const geometry_msgs::Pose& p,
                                  moveit_msgs::CollisionObject& co,
                                  int object_operation = moveit_msgs::CollisionObject::ADD);

moveit_msgs::CollisionObject savedSTLToCollisionObjectMsg(const std::string& file_path,
                                                          const Eigen::Vector3d& scale_vector,
                                                          const std::string& frame_id,
                                                          const geometry_msgs::Pose& p,
                                                          int object_operation = moveit_msgs::CollisionObject::ADD);

void savedSTLToCollisionObjectMsg(const std::string& file_path, const Eigen::Vector3d& scale_vector,
                                  const std::string& frame_id, const std::string& obj_id,
                                  const geometry_msgs::Pose& p,
                                  moveit_msgs::CollisionObject& co,
                                  int object_operation = moveit_msgs::CollisionObject::ADD);

// obj id is the filename
moveit_msgs::AttachedCollisionObject savedSTLToAttachedCollisionObjectMsg(const std::string& file_path,
                                                                          const Eigen::Vector3d& scale_vector,
                                                                          const std::string& frame_id,
                                                                          const std::string& link_id,
                                                                          const geometry_msgs::Pose& p,
                                                                          int object_operation = moveit_msgs::CollisionObject::ADD);

void savedSTLToAttachedCollisionObjectMsg(const std::string& file_path,
                                          const Eigen::Vector3d& scale_vector,
                                          const std::string& frame_id,
                                          const std::string& link_id,
                                          const geometry_msgs::Pose& p,
                                          moveit_msgs::AttachedCollisionObject& co,
                                          int object_operation = moveit_msgs::CollisionObject::ADD);

void savedSTLToAttachedCollisionObjectMsg(const std::string& file_path,
                                          const Eigen::Vector3d& scale_vector,
                                          const std::string& frame_id,
                                          const std::string& link_id,
                                          const std::string& obj_id,
                                          const geometry_msgs::Pose& p,
                                          moveit_msgs::AttachedCollisionObject& co,
                                          int object_operation = moveit_msgs::CollisionObject::ADD);
}

#endif
