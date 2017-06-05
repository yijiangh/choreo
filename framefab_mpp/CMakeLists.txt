# following the overall structure and ordering
# indicated by ros cmake: http://wiki.ros.org/catkin/CMakeLists.txt

cmake_minimum_required(VERSION 2.8.3)
project(framefab_mpp)

add_definitions("-std=c++11")

## Find catkin macros and libraries (build-depend)
find_package(catkin REQUIRED COMPONENTS
        rviz
        moveit_core
        moveit_ros_planning
        moveit_ros_planning_interface
        moveit_visual_tools
        descartes_core
        descartes_moveit
        descartes_trajectory
        descartes_planner REQUIRED
        trajectory_msgs
        framefab_msgs
        moveit_ros_planning_interface
        )

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/modules")

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED)
find_package(Eigen REQUIRED)

## Message/Service/Action Generators

## Invoke Message/Service/Action Generation

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
        tf_conversions
        eigen_conversions
        moveit_core
        moveit_ros_planning
        moveit_ros_planning_interface
        moveit_msgs
        moveit_visual_tools
        framefab_msgs
        DEPENDS
        boost
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN_INCLUDE_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS})

## This setting causes Qt's "MOC" generation to happen automatically.
set(CMAKE_AUTOMOC ON)

## This plugin includes Qt widgets, so we must include Qt.
## We'll use the version that rviz used so they are compatible.
if(rviz_QT_VERSION VERSION_LESS "5")
    message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
    find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
    ## pull in all required include dirs, define QT_LIBRARIES, etc.
    include(${QT_USE_FILE})
else()
    message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
    find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
    ## make target_link_libraries(${QT_LIBRARIES}) pull in all required dependencies
    set(QT_LIBRARIES Qt5::Widgets)
endif()

add_definitions(-DQT_NO_KEYWORDS)

###########
## Build ##
###########
## Specifying build targets

### ------------------ declare cpp library ------------------ ###

## The generated MOC files are included automatically as headers.
## **header and source files should be added to library at the same time**
## otherwise AUTO_MOC cannot find header file successfully even you include
## the directory.

set(HEADER_FILES
        include/framefab_rviz_panel.h
        include/framefab_render_widget.h
        )

set(SRC_FILES
        src/framefab_rviz_panel.cpp
        src/framefab_render_widget.cpp
        src/wire_frame/wire_frame_line_graph.cpp
        src/wire_frame/wire_frame_collision_objects.cpp
        src/framefab_planner.cpp
        )

add_library(${PROJECT_NAME} ${SRC_FILES} ${HEADER_FILES})

target_link_libraries(
        ${PROJECT_NAME} ${QT_LIBRARIES} ${catkin_LIBRARIES}
)

### ------------------ end library ------------------ ###

## ------------------ declare a cpp executable ------------------ ###
add_executable(
        ${PROJECT_NAME}_node
        src/framefab_mpp_node.cpp
        src/framefab_planner.cpp
)

target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})

### ------------------ end executable ------------------ ###

###########
## Installation ##
###########
## After build time, targets are placed into the devel space of the catkin workspace. However, often we want to install
## targets to the system so that they can be used by others or to a local folder to test a system-level installation.
## In other words, if you want to be able to do a "make install" of your code, you need to specify where targets should
## end up.

install(TARGETS
        ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

#install(FILES
#        framefab_rviz_panel_package_description.xml
#        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
