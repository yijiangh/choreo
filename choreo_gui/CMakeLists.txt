cmake_minimum_required(VERSION 2.8.3)
project(choreo_gui)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
	choreo_msgs
	roscpp
	rviz
	)

find_package(Qt5Widgets REQUIRED)
find_package(Qt5Core REQUIRED)
find_package(Qt5Gui REQUIRED)

catkin_package(
	CATKIN_DEPENDS 
	choreo_msgs
	roscpp 
	rviz
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

set(choreo_gui_SRCS
	#core
	src/choreo_panel.cpp
	src/choreo_widget.cpp
	#state
	src/states/system_init_state.cpp
	src/states/task_sequence_processing_state.cpp
	src/states/select_tasks_state.cpp
	src/states/process_planning_state.cpp
	src/states/select_plan_state.cpp
	#params
	src/params_submenu.cpp
	src/parameter_window_base.cpp
	src/params/model_input_config_widget.cpp
	src/params/task_sequence_input_config_widget.cpp
	src/params/robot_input_config_widget.cpp
	src/params/output_save_dir_input_config_widget.cpp
	#selection
	src/selection/selection_widget.cpp
	src/selection/select_for_plan_pop_up_widget.cpp
	)

set(choreo_gui_HDRS
	include/choreo_gui/choreo_panel.h
	include/choreo_gui/choreo_widget.h
	#states
	include/choreo_gui/gui_state.h
	include/choreo_gui/states/system_init_state.h
	include/choreo_gui/states/task_sequence_processing_state.h
	include/choreo_gui/states/select_tasks_state.h
	include/choreo_gui/states/process_planning_state.h
	include/choreo_gui/states/select_plan_state.h
	#params
	include/choreo_gui/params_submenu.h
	include/choreo_gui/parameter_window_base.h
	include/choreo_gui/params/model_input_config_widget.h
	include/choreo_gui/params/task_sequence_input_config_widget.h
	include/choreo_gui/params/robot_input_config_widget.h
	include/choreo_gui/params/output_save_dir_input_config_widget.h
	#selection
	include/choreo_gui/selection/selection_widget.h
	include/choreo_gui/selection/select_for_plan_pop_up_widget.h
	)

set(choreo_gui_UIS
	src/uis/choreo_widget.ui
	src/uis/params_submenu.ui
	src/uis/model_input_config.ui
	src/uis/task_sequence_input_config.ui
	src/uis/robot_input_config.ui
	src/uis/output_save_dir_input_config.ui
	src/uis/selection_widget.ui
	src/uis/select_for_plan_pop_up.ui
	)

set(choreo_gui_RESOURCES
	src/uis/robot_input_widget_resource.qrc
	)

set(choreo_gui_INCLUDE_DIRECTORIES
	include
	${CMAKE_CURRENT_BINARY_DIR}
	)

include_directories(include)
include_directories(
	${catkin_INCLUDE_DIRS}
	${choreo_gui_INCLUDE_DIRECTORIES}
)

add_definitions(-DQT_NO_KEYWORDS)
qt5_add_resources(choreo_gui_RESOURCES_H ${choreo_gui_RESOURCES})
qt5_wrap_cpp(choreo_gui_MOCS ${choreo_gui_HDRS})
qt5_wrap_ui(choreo_gui_UIS_H ${choreo_gui_UIS})

## Declare choreo_gui library
add_library(${PROJECT_NAME}
	${choreo_gui_UIS_H}
	${choreo_gui_MOCS}
	${choreo_gui_SRCS}
	${choreo_gui_RESOURCES_H}
	)

target_link_libraries(${PROJECT_NAME}
	${catkin_LIBRARIES}
	Qt5::Widgets
	Qt5::Gui
	)

add_dependencies(${PROJECT_NAME} choreo_msgs_generate_messages_cpp)

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
