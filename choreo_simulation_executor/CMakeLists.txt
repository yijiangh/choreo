cmake_minimum_required(VERSION 2.8.3)
project(choreo_simulation_execution)

find_package(catkin REQUIRED COMPONENTS
  choreo_msgs
  roscpp
  actionlib
)

catkin_package(
  CATKIN_DEPENDS
    actionlib
    choreo_msgs
    roscpp 
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(simulation_execution_service_node
	src/simulation_execution_service_node.cpp
	src/simulation_execution_service.cpp
)

add_dependencies(simulation_execution_service_node choreo_msgs_generate_messages_cpp)
add_dependencies(simulation_execution_service_node choreo_industrial_robot_simulator_service_generate_messages_cpp)

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
