cmake_minimum_required(VERSION 2.8.3)
project(framefab_simulation_execution)

find_package(catkin REQUIRED COMPONENTS
  framefab_msgs
  roscpp
  framefab_industrial_robot_simulator_service
  actionlib
)

catkin_package(
  CATKIN_DEPENDS
    actionlib
    framefab_msgs
    roscpp 
    framefab_industrial_robot_simulator_service
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(simulation_execution_service_node
	src/simulation_execution_service_node.cpp
	src/simulation_execution_service.cpp
)

add_dependencies(simulation_execution_service_node framefab_msgs_generate_messages_cpp)
add_dependencies(simulation_execution_service_node framefab_industrial_robot_simulator_service_generate_messages_cpp)

target_link_libraries(simulation_execution_service_node
  ${catkin_LIBRARIES}
)

install(TARGETS simulation_execution_service_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
