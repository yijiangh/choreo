cmake_minimum_required(VERSION 2.8.3)
project(kuka_kr10_support)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  pluginlib
  cmake_modules
  geometric_shapes
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED system filesystem date_time thread)
find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
find_package(Eigen REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
	INCLUDE_DIRS
		include
  CATKIN_DEPENDS
    moveit_core
    moveit_ros_planning_interface
    interactive_markers
)

###########
## Build ##
###########
add_definitions(-DQT_NO_KEYWORDS)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIR}
        ${EIGEN_INCLUDE_DIRS}
        SYSTEM
)

include(${QT_USE_FILE})

#if (CATKIN_ENABLE_TESTING)
#  find_package(roslaunch REQUIRED)
#  roslaunch_add_file_check(test/roslaunch_test_kr10r1100sixx.xml)
#endif()

#install(DIRECTORY config launch meshes urdf
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)


#link_directories(${catkin_LIBRARY_DIRS})

qt4_wrap_cpp(MOC_FILES
        include/FrameFabRvizPanel.h
	    include/FrameFabRenderWidget.h
)

# declare a cpp library
add_library(${PROJECT_NAME}_lib 
	${MOC_FILES}
	src/FrameFab.cpp
	src/FrameFabRenderWidget.cpp
	src/FrameFabRvizPanel.cpp
    src/WireFrame.cpp
)

# declare a cpp executable
add_executable(
	${PROJECT_NAME} 
	src/framefab_node.cpp
)

target_link_libraries(${PROJECT_NAME}
	${PROJECT_NAME}_lib
)
# ${catkin_LIBRARIES} ${Boost_LIBRARIES}

#install(TARGETS ${PROJECT_NAME} DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
#install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
