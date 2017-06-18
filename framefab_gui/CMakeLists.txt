cmake_minimum_required(VERSION 2.8.3)
project(framefab_gui)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
	framefab_msgs
	roscpp
	rviz
	)

message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
include(${QT_USE_FILE})

catkin_package(
	CATKIN_DEPENDS framefab_msgs roscpp rviz
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
	src/states/model_input_state.cpp
	src/states/system_init_state.cpp
	# params
	src/params_submenu.cpp
	src/parameter_window_base.cpp
	src/params/model_input_config_widget.cpp
	)

set(framefab_gui_HDRS
	include/framefab_gui/framefab_panel.h
	include/framefab_gui/framefab_widget.h
	#states
	include/framefab_gui/gui_state.h
	include/framefab_gui/states/model_input_state.h
	include/framefab_gui/states/system_init_state.h
	#params
	include/framefab_gui/params_submenu.h
	include/framefab_gui/parameter_window_base.h
	include/framefab_gui/params/model_input_config_widget.h
	)

set(framefab_gui_UIS
	src/uis/framefab_widget.ui
	src/uis/params_submenu.ui
	src/uis/model_input_config.ui
	)

set(framefab_gui_INCLUDE_DIRECTORIES
	include
	${CMAKE_CURRENT_BINARY_DIR}
	)

add_definitions(-DQT_NO_KEYWORDS)
qt4_wrap_cpp(framefab_gui_MOCS ${framefab_gui_HDRS})
qt4_wrap_ui(framefab_gui_UIS_H ${framefab_gui_UIS})

include_directories(include)
include_directories(
	${catkin_INCLUDE_DIRS}
	${framefab_gui_INCLUDE_DIRECTORIES}
)

## Declare a cpp library
add_library(${PROJECT_NAME}
	${framefab_gui_UIS_H}
	${framefab_gui_MOCS}
	${framefab_gui_SRCS}
	)

target_link_libraries(${PROJECT_NAME}
	${catkin_LIBRARIES}
	${QT_LIBRARIES} ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY}
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
