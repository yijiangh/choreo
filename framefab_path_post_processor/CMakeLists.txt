cmake_minimum_required(VERSION 2.8.3)
project(framefab_path_post_processor)
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  framefab_msgs
)

catkin_package(
  INCLUDE_DIRS 
    include
  CATKIN_DEPENDS 
    framefab_msgs
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

add_dependencies(kuka_framefab_process_service_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_dependencies(framefab_process_service_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
