cmake_minimum_required(VERSION 2.8.3)
project(framefab_task_sequence_planning)

find_package(catkin REQUIRED COMPONENTS
	cmake_modules
	geometric_tools_engine
	)

# add cmake module files for dependent packages
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules")

#--- Find Eigen
find_package(Eigen REQUIRED)

#--- Find MOSEK
find_package(MOSEK REQUIRED)

#--- Find LAPACK
find_package(LAPACK REQUIRED)

add_definitions("-std=c++11 -D__LINUX__ -DGTE_DEV_OPENGL -DGTE_DISABLE_PCH")
add_definitions("-Wno-write-strings")

###################################
## catkin specific configuration ##
###################################
catkin_package(
	INCLUDE_DIRS include
	LIBRARIES ${PROJECT_NAME}
	CATKIN_DEPENDS geometric_tools_engine cmake_modules
	DEPENDS Eigen MOSEK LAPACK
	CFG_EXTRAS framefab_core_cmake_modules-extra.cmake
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${Eigen_INCLUDE_DIRS}
	${MOSEK_INCLUDE_DIRS}
)

set(${PROJECT_NAME}_CUT_GENERATORS_SRC_FILES
	src/cut_generators/ADMMCut.cpp
	src/cut_generators/GraphCut.cpp
	src/cut_generators/NormalCut.cpp
	src/cut_generators/QP.cpp
	src/cut_generators/QPFactory.cpp
	src/cut_generators/QPMosek.cpp
	)

set(${PROJECT_NAME}_SEQUENCE_ANALYZERS_SRC_FILES
	src/sequence_analyzers/BFAnalyzer.cpp
	src/sequence_analyzers/FFAnalyzer.cpp
	src/sequence_analyzers/ProcAnalyzer.cpp
	src/sequence_analyzers/SeqAnalyzer.cpp
	)

set(${PROJECT_NAME}_UTILS_SRC_FILES
	src/utils/CoordTrans.cpp
	src/utils/DualGraph.cpp
	src/utils/ExtruderCone.cpp
	src/utils/IllCondDetector.cpp
	src/utils/Loader.cpp
	src/utils/QuadricCollision.cpp
	src/utils/ResolveAngle.cpp
	src/utils/Statistics.cpp
	src/utils/Stiffness.cpp
	src/utils/StiffnessIO.cpp
	src/utils/StiffnessSolver.cpp
	src/utils/Timer.cpp
	src/utils/WireFrame.cpp
	)

### Declare a C++ library
add_library(${PROJECT_NAME}
	${${PROJECT_NAME}_CUT_GENERATORS_SRC_FILES}
	${${PROJECT_NAME}_SEQUENCE_ANALYZERS_SRC_FILES}
	${${PROJECT_NAME}_UTILS_SRC_FILES}
	src/FiberPrintPARM.cpp
	src/FiberPrintPlugIn.cpp
	)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
#${LAPACK_LIBRARIES} ${MOSEK_LIBRARIES}
