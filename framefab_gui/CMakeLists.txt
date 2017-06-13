cmake_minimum_required(VERSION 2.8.3)
project(framefab_rviz_plugin)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  framefab_msgs
  roscpp
  rviz
)

find_package(Qt5Widgets REQUIRED)
find_package(Qt5Core REQUIRED)
find_package(Qt5Gui REQUIRED)

add_definitions(-DQT_NO_KEYWORDS)

catkin_package(
  CATKIN_DEPENDS framefab_msgs roscpp rviz
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

set(framefab_rviz_plugin_SRCS
# core
 src/blending_panel.cpp
 src/blending_widget.cpp
)

set(framefab_rviz_plugin_HDRS
	include/framefab_gui/blending_panel.h
	include/framefab_gui/blending_widget.h
)

set(framefab_rviz_plugin_UIS
 src/uis/blending_widget.ui
)

set(framefab_rviz_plugin_INCLUDE_DIRECTORIES
  include
  ${CMAKE_CURRENT_BINARY_DIR}
)

add_definitions(-DQT_NO_KEYWORDS)
qt5_wrap_cpp(framefab_rviz_plugin_MOCS ${framefab_rviz_plugin_HDRS})
qt5_wrap_ui(framefab_rviz_plugin_UIS_H ${framefab_rviz_plugin_UIS})

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${framefab_rviz_plugin_INCLUDE_DIRECTORIES}
)

## Declare a cpp library
add_library(${PROJECT_NAME}
  ${framefab_rviz_plugin_UIS_H}
  ${framefab_rviz_plugin_MOCS}
  ${framefab_rviz_plugin_SRCS}
)

target_link_libraries(${PROJECT_NAME} 
                      ${catkin_LIBRARIES}
                      Qt5::Widgets
                      Qt5::Gui 
                      )

add_dependencies(${PROJECT_NAME} framefab_msgs_generate_messages_cpp)

find_package(class_loader)
class_loader_hide_library_symbols(${PROJECT_NAME})

# Install
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
