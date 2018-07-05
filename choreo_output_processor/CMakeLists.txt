cmake_minimum_required(VERSION 2.8.3)
project(choreo_output_processor)
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
	geometry_msgs
	moveit_msgs
	trajectory_msgs
	choreo_msgs
	tf_conversions
	eigen_conversions
	choreo_rapidjson
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS 
	include
  CATKIN_DEPENDS
	geometry_msgs
	moveit_msgs
	trajectory_msgs
	choreo_msgs
	tf_conversions
	eigen_conversions
	choreo_rapidjson
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
add_library(choreo_output_processor
	src/choreo_output_processor.cpp
	)
target_link_libraries(choreo_output_processor
	${catkin_LIBRARIES}
	${Eigen_LIBRARIES}
	)

add_executable(choreo_output_processor_node
	src/choreo_output_processor_node.cpp
)

target_link_libraries(choreo_output_processor_node
	choreo_output_processor
	)

#add_dependencies(choreo_output_processor_node choreo_msgs_generate_messages_cpp)
