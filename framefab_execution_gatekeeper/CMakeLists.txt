cmake_minimum_required(VERSION 2.8.3)
project(framefab_execution_gatekeeper)
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  framefab_msgs
  trajectory_msgs
)

catkin_package(
  INCLUDE_DIRS 
    include
  CATKIN_DEPENDS 
    framefab_msgs
    trajectory_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(framefab_execution_gatekeeper_service_node
	src/framefab_execution_gatekeeper_service_node.cpp
  src/framefab_execution_gatekeeper_service.cpp
	src/execution_utils.cpp
)

add_dependencies(framefab_execution_gatekeeper_service_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(framefab_execution_gatekeeper_service_node
  ${catkin_LIBRARIES}
)
