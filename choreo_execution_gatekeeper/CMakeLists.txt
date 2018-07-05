cmake_minimum_required(VERSION 2.8.3)
project(choreo_execution_gatekeeper)
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  choreo_msgs
  trajectory_msgs
)

catkin_package(
  INCLUDE_DIRS 
    include
  CATKIN_DEPENDS 
    choreo_msgs
    trajectory_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(choreo_execution_gatekeeper_service_node
	src/choreo_execution_gatekeeper_service_node.cpp
  src/choreo_execution_gatekeeper_service.cpp
	src/execution_utils.cpp
)

add_dependencies(choreo_execution_gatekeeper_service_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(choreo_execution_gatekeeper_service_node
  ${catkin_LIBRARIES}
)
