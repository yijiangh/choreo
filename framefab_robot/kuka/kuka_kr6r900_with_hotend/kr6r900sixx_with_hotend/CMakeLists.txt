cmake_minimum_required(VERSION 2.8.3)

project(kr6r900sixx_with_hotend)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY meshes urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
