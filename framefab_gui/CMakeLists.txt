cmake_minimum_required(VERSION 2.8.3)
project(framefab_gui)

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/modules")

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
	framefab_msgs
	roscpp
	rviz
	)

#--- Find GL & GLUT
find_package(GL REQUIRED)
find_package(GLUT REQUIRED)

find_package(Qt5Widgets REQUIRED)
find_package(Qt5Core REQUIRED)
find_package(Qt5Gui REQUIRED)
find_package(Qt5OpenGL REQUIRED)

catkin_package(
	CATKIN_DEPENDS 
	framefab_msgs
	roscpp 
	rviz
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

set(framefab_gui_SRCS
	# core
	src/framefab_panel.cpp
	src/framefab_widget.cpp
	# state
	src/states/path_planning_state.cpp
	src/states/system_init_state.cpp
	# params
	src/params_submenu.cpp
	src/parameter_window_base.cpp
	src/params/model_input_config_widget.cpp
	# input_ui
	src/input_ui/mainwindow.cpp
	src/input_ui/renderingwidget.cpp
	src/input_ui/WireFrame.cpp
	src/input_ui/ArcBall.cpp
	)

set(framefab_gui_HDRS
	include/framefab_gui/framefab_panel.h
	include/framefab_gui/framefab_widget.h
	#states
	include/framefab_gui/gui_state.h
	include/framefab_gui/states/path_planning_state.h
	include/framefab_gui/states/system_init_state.h
	#params
	include/framefab_gui/params_submenu.h
	include/framefab_gui/parameter_window_base.h
	include/framefab_gui/params/model_input_config_widget.h
	#input_ui
	include/framefab_gui/input_ui/mainwindow.h
	include/framefab_gui/input_ui/renderingwidget.h
	include/framefab_gui/input_ui/Vec.h
	include/framefab_gui/input_ui/ArcBall.h
	include/framefab_gui/input_ui/WireFrame.h
	)

set(framefab_gui_UIS
	src/uis/framefab_widget.ui
	src/uis/params_submenu.ui
	src/uis/model_input_config.ui
	src/uis/framefab_mainwindow.ui
	)

set(framefab_gui_INCLUDE_DIRECTORIES
	include
	${CMAKE_CURRENT_BINARY_DIR}
	)

#set(RESOURCE_FOLDER src/res)
#
#set(RESOURCES_FILES
#	src/uis/mainwindow.qrc)

include_directories(include)
include_directories(
	${catkin_INCLUDE_DIRS}
	${framefab_gui_INCLUDE_DIRECTORIES}
)

add_definitions(-DQT_NO_KEYWORDS)
qt5_wrap_cpp(framefab_gui_MOCS ${framefab_gui_HDRS})
qt5_wrap_ui(framefab_gui_UIS_H ${framefab_gui_UIS})

#add_library(input_ui
#  ${input_ui_UIS_H}
#  ${input_ui_MOCS}
#  ${input_ui_SRCS}
#  src/input_ui/WireFrame.cpp
#)
#
#target_link_libraries(input_ui
#	Qt5::Widgets
#	Qt5::Gui
#	Qt5::OpenGL
#)
#
#add_dependencies(input_ui framefab_msgs_generate_messages_cpp)


## Declare framefab_gui library
add_library(${PROJECT_NAME}
	${framefab_gui_UIS_H}
	${framefab_gui_MOCS}
	${framefab_gui_SRCS}
	)

target_link_libraries(${PROJECT_NAME}
	${catkin_LIBRARIES}
	Qt5::Widgets
	Qt5::Gui
	Qt5::OpenGL
	${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY} ${GLUT_LIBRARIES}
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
