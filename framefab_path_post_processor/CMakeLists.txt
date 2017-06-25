cmake_minimum_required(VERSION 2.8.3)
project(framefab_path_post_processor)
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
	framefab_msgs
	geometry_msgs
	cmake_modules
)

find_package(RapidJSON REQUIRED)
find_package(Eigen REQUIRED)

generate_messages(DEPENDENCIES framefab_msgs geometry_msgs)

catkin_package(
  INCLUDE_DIRS 
    include
  CATKIN_DEPENDS
    framefab_msgs
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
	${Eigen_INCLUDE_DIRS}
	${RapidJSON_INCLUDE_DIRS}
)

##_____________
## Libraries ##

# PathPostProcessor library
#add_library()
#target_link_libraries()

add_executable(framefab_path_post_processor_node
	framefab_path_post_processor_node.cpp
)

add_dependencies(framefab_path_post_processor_node framefab_msgs_generate_messages_cpp)
