// register IKFastKinematicsPlugin as a KinematicsBase implementation
#include <framefab_kr6_r900_ikfast_manipulator_plugin/kuka_kr6r900sixx_manipulator_ikfast_moveit_plugin.hpp>
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(framefab_kr6_r900_ikfast_manipulator_plugin::IKFastKinematicsPlugin,
                       kinematics::KinematicsBase);
