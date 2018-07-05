cmake_minimum_required(VERSION 2.8.3)
project(choreo_task_sequence_processor)
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
	geometry_msgs
	moveit_msgs
	choreo_msgs
	tf_conversions
	eigen_conversions
	choreo_rapidjson
	choreo_geometry_conversion_helpers
)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem)

catkin_package(
  INCLUDE_DIRS 
	include
	#
  CATKIN_DEPENDS
	geometry_msgs
	moveit_msgs
	choreo_msgs
	tf_conversions
	eigen_conversions
	choreo_rapidjson
	choreo_geometry_conversion_helpers
	#
	DEPENDS
	Boost
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
	${Eigen_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
)

##_____________
## Libraries ##
# PathPostProcessor library
add_library(task_sequence_processor
	src/task_sequence_processor.cpp
	src/unit_process.cpp
	)

add_library(json2msg_helper
	src/json2msg_helpers.cpp
	)

target_link_libraries(json2msg_helper
	${catkin_LIBRARIES}
	${Eigen_LIBRARIES}
	)

target_link_libraries(task_sequence_processor
	${catkin_LIBRARIES}
	${Eigen_LIBRARIES}
	json2msg_helper
	)

add_executable(task_sequence_processor_node
	src/task_sequence_processor_node.cpp
)

target_link_libraries(task_sequence_processor_node
	task_sequence_processor
	)

add_dependencies(task_sequence_processor_node choreo_msgs_generate_messages_cpp)
