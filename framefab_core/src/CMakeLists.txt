
# add folder for findModule and qt make supportive functions
list(APPEND CMAKE_MODULE_PATH "$cmake/modules")

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -D__LINUX__ -DGTE_DEV_OPENGL -DGTE_DISABLE_PCH")

#--- Find Eigen
find_package(Eigen3 REQUIRED)

#--- Find MOSEK
find_package(MOSEK REQUIRED)

#--- Find LAPACK
find_package(LAPACK REQUIRED)

set(GTENGINE_INCLUDE_DIRS ${CMAKE_SOURCE_DIR}/ext/GTEngine/Include)

include_directories(
#  SYSTEM
  ${Eigen_INCLUDE_DIRS}
  ${MOSEK_INCLUDE_DIRS}
  ${GTENGINE_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}
  ${CMAKE_SOURCE_DIR}/src
  ${CMAKE_SOURCE_DIR}/ext
)

 #if (NOT APPLE)
 #   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fvisibility=hidden")
 #endif()

## http://stackoverflow.com/questions/3201154/cmake-automatically-add-all-files-in-a-folder-to-a-target
file(GLOB FRAMEFAB_CORE_SRC ${CMAKE_SOURCE_DIR}/include/framefab_core/*.h ${CMAKE_SOURCE_DIR}/src/framefab_core/*.cpp)

add_library(framefab_core ${FRAMEFAB_CORE_SRC})

target_link_libraries(framefab_core ${LAPACK_LIBRARIES} ${MOSEK_LIBRARIES} GTEngine)
