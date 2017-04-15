cmake_minimum_required(VERSION 3.0)
project(framefab_mpp)

# output CXX compiler info
message(STATUS "CMAKE_SYSTEM_NAME          = ${CMAKE_SYSTEM_NAME}"         )
message(STATUS "CMAKE_CXX_COMPILER_ID      = ${CMAKE_CXX_COMPILER_ID}"     )
message(STATUS "CMAKE_CXX_COMPILER_VERSION = ${CMAKE_CXX_COMPILER_VERSION}")
message(STATUS "CMAKE_COMPILER_IS_GNUCXX   = ${CMAKE_COMPILER_IS_GNUCXX}"  )

# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -D__LINUX__ -DGTE_DEV_OPENGL -DGTE_DISABLE_PCH")

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


# link_directories(${catkin_LIBRARY_DIRS})

qt4_wrap_cpp(MOC_FILES
        include/FrameFabRvizPanel.h
	    include/FrameFabRenderWidget.h
)

# declare a cpp library
add_library(lib${PROJECT_NAME}
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
	lib${PROJECT_NAME} ${catkin_LIBRARIES}
)
#${Boost_LIBRARIES}

#install(TARGETS ${PROJECT_NAME} DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
#install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
