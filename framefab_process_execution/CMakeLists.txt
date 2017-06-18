cmake_minimum_required(VERSION 2.8.3)
project(framefab_process_execution)
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
#  kuka_file_suite
  framefab_msgs
  industrial_robot_simulator_service
  trajectory_msgs
)

catkin_package(
  INCLUDE_DIRS 
    include
  CATKIN_DEPENDS 
#    kuka_file_suite
    framefab_msgs
    industrial_robot_simulator_service
    trajectory_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(kuka_framefab_process_service_node 
  src/kuka_framefab_process_service_node.cpp 
  src/kuka_framefab_process_service.cpp
  src/process_utils.cpp
)

add_executable(framefab_process_service_node
  src/framefab_process_service_node.cpp 
  src/framefab_process_service.cpp 
  src/process_utils.cpp
	src/kuka_framefab_process_service.cpp src/process_utils.h src/process_utils.cpp)

add_dependencies(kuka_framefab_process_service_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_dependencies(framefab_process_service_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(kuka_framefab_process_service_node
  ${catkin_LIBRARIES}
)

target_link_libraries(framefab_process_service_node
  ${catkin_LIBRARIES}
)
