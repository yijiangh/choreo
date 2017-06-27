cmake_minimum_required(VERSION 2.8.3)
project(framefab_path_post_processor)
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
	geometry_msgs
	moveit_msgs
	framefab_msgs
	tf_conversions
	eigen_conversions
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
)

###########
## Build ##
###########

include_directories(
  include
	ext/rapidjson/include
  ${catkin_INCLUDE_DIRS}
	${Eigen_INCLUDE_DIRS}
)

##_____________
## Libraries ##
# PathPostProcessor library
add_library(path_post_processor
	src/path_post_processor.cpp
	src/process_path.cpp
	)
target_link_libraries(path_post_processor
	${catkin_LIBRARIES}
	${Eigen_LIBRARIES}
	)

add_executable(path_post_processor_node
	src/path_post_processor_node.cpp
)

target_link_libraries(path_post_processor_node
	path_post_processor
	)

add_dependencies(path_post_processor_node framefab_msgs_generate_messages_cpp)
