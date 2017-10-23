cmake_minimum_required(VERSION 2.8.3)
project(framefab_task_sequence_processor)
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
add_library(task_sequence_processor
	src/task_sequence_processor.cpp
	src/unit_process.cpp
	)
target_link_libraries(task_sequence_processor
	${catkin_LIBRARIES}
	${Eigen_LIBRARIES}
	)

add_executable(task_sequence_processor_node
	src/task_sequence_processor_node.cpp
)

target_link_libraries(task_sequence_processor_node
	task_sequence_processor
	)

add_dependencies(task_sequence_processor_node framefab_msgs_generate_messages_cpp)
