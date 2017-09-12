cmake_minimum_required(VERSION 2.8.3)
project(framefab_output_processor)
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
	geometry_msgs
	moveit_msgs
	framefab_msgs
	tf_conversions
	eigen_conversions
	framefab_rapidjson
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS 
	include
  CATKIN_DEPENDS
	geometry_msgs
	moveit_msgs
	framefab_msgs
	tf_conversions
	eigen_conversions
	framefab_rapidjson
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
	${Eigen_INCLUDE_DIRS}
)

##_____________
## Libraries ##
# PathPostProcessor library
add_library(framefab_output_processor
	src/framefab_output_processor.cpp
	)
target_link_libraries(framefab_output_processor
	${catkin_LIBRARIES}
	${Eigen_LIBRARIES}
	)

add_executable(framefab_output_processor_node
	src/framefab_output_processor_node.cpp
)

target_link_libraries(framefab_output_processor_node
	framefab_output_processor
	)

#add_dependencies(framefab_output_processor_node framefab_msgs_generate_messages_cpp)