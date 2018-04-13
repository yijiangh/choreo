cmake_minimum_required(VERSION 2.8.11)
project(choreo_descartes_planner)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  descartes_core
  descartes_trajectory
  descartes_planner
  moveit_core
  roscpp
  pluginlib
  cmake_modules
  rviz_visual_tools
  eigen_conversions
  moveit_msgs
  )

find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)

# Eigen 3.2 (Wily) only provides EIGEN3_INCLUDE_DIR, not EIGEN3_INCLUDE_DIRS
if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS
  include
  LIBRARIES
  ${PROJECT_NAME}
  CATKIN_DEPENDS
  roscpp
  moveit_core
  #
  descartes_core
  descartes_trajectory
  descartes_planner
  #
  eigen_conversions
  moveit_msgs
  DEPENDS
  Boost
  EIGEN3
)

###########
## Build ##
###########
include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  )

add_library(${PROJECT_NAME}
  src/choreo_ladder_graph_builder.cpp
  src/capsulated_ladder_tree_RRTstar.cpp
	include/choreo_descartes_planner/pose_sampling_helpers.cpp
  src/pose_verification_helpers.cpp
  )

target_link_libraries(
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

############
# Install ##
############
install(
  TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
