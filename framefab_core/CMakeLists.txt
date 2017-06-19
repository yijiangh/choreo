cmake_minimum_required(VERSION 2.8.3)
project(framefab_core)
ADD_DEFINITIONS( -std=c++11 )

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  interactive_markers
  moveit_ros_move_group
  moveit_ros_planning_interface
  tf
  tf_conversions
#  framefab_process_path_generation
  framefab_msgs
)

find_package(Boost REQUIRED COMPONENTS system)

find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
  message(STATUS "OPENMP FOUND")
  set(OpenMP_FLAGS ${OpenMP_CXX_FLAGS})  # or if you use C: ${OpenMP_C_FLAGS}
  set(OpenMP_LIBS gomp)
endif()

catkin_package(
  INCLUDE_DIRS 
    include
  LIBRARIES 
    ${PROJECT_NAME}
  CATKIN_DEPENDS 
    roscpp
    sensor_msgs
    interactive_markers
    tf
    tf_conversions
#    framefab_process_path_generation
    framefab_msgs
)


###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(${PROJECT_NAME} 
  src/framefab_core_service.cpp
)

target_compile_options(${PROJECT_NAME} PRIVATE ${OpenMP_FLAGS})

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} yaml-cpp)
add_dependencies(${PROJECT_NAME} framefab_msgs_generate_messages_cpp)

## surface detection service
add_executable(framefab_core_service  src/framefab_core_service.cpp)
#                                      src/services/blending_service_path_generation.cpp)
target_link_libraries(framefab_core_service ${PROJECT_NAME})
add_dependencies(framefab_core_service framefab_msgs_generate_messages_cpp)
target_compile_options(framefab_core_service PRIVATE ${OpenMP_FLAGS})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
