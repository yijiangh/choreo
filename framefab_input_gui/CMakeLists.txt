cmake_minimum_required(VERSION 2.8.3)
project(framefab_input_gui)
ADD_DEFINITIONS( -std=c++11 )

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/modules")

find_package(catkin REQUIRED COMPONENTS
  framefab_msgs
  roscpp
)

#--- Find GL & GLUT
find_package(GL REQUIRED)
find_package(GLUT REQUIRED)

#--- Find Qt
find_package(Qt5Core REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(Qt5Gui REQUIRED)
find_package(Qt5OpenGL REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS framefab_msgs roscpp
)

###########
## Build ##
###########

set(framefab_input_gui_SRCS
	src/mainwindow.cpp
	src/renderingwidget.cpp
	src/WireFrame.cpp
	src/ArcBall.cpp
	src/framefab_input_gui.cpp
	src/framefab_input_gui_node.cpp
)

set(framefab_input_gui_HDRS
	include/framefab_input_gui/mainwindow.h
	include/framefab_input_gui/renderingwidget.h
	include/framefab_input_gui/Vec.h
	include/framefab_input_gui/ArcBall.h
	include/framefab_input_gui/WireFrame.h
	include/framefab_input_gui/framefab_input_gui.h
)

set(framefab_input_gui_UIS
	src/uis/framefab_mainwindow.ui
)

set(framefab_input_gui_INCLUDE_DIRECTORIES
  include
  ${CMAKE_CURRENT_BINARY_DIR}
)

add_definitions(-DQT_NO_KEYWORDS)
qt5_wrap_cpp(framefab_input_gui_MOCS ${framefab_input_gui_HDRS})
qt5_wrap_ui(framefab_input_gui_UIS_H ${framefab_input_gui_UIS})

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${framefab_input_gui_INCLUDE_DIRECTORIES}
)

## Declare a cpp executable
add_executable(framefab_input_gui_node
  ${framefab_input_gui_UIS_H}
  ${framefab_input_gui_MOCS}
  ${framefab_input_gui_SRCS}
)

target_link_libraries(framefab_input_gui_node
	${catkin_LIBRARIES}
	Qt5::Widgets
	Qt5::Gui
	Qt5::OpenGL
	${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY} ${GLUT_LIBRARIES}
)

add_dependencies(framefab_input_gui_node framefab_msgs_generate_messages_cpp)

# Install
install(TARGETS framefab_input_gui_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
