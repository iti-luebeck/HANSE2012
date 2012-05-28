cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


# We use our own FindEigen3.cmake file to support electric and fuerte
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})
add_definitions(${EIGEN3_DEFINITIONS})

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

add_definitions(-std=c++0x -isystem ${PROJECT_SOURCE_DIR}/src/trng -O3 -fno-math-errno -funsafe-math-optimizations -fno-rounding-math -fno-signaling-nans -mfpmath=sse)

rosbuild_add_executable(sonar_wall_detection
  src/wall_detection.cpp
)

rosbuild_add_executable(sonar_online_wall_detection
  src/online_wall_detection.cpp
)

file(GLOB TRNG_FILES src/trng/trng/*.cc)

rosbuild_add_executable(sonar_localization
  src/localization.cpp
  src/world_map.cpp
  src/particle_filter.cpp
  src/util.cpp
  ${TRNG_FILES}
)

find_package(OpenCV REQUIRED)

target_link_libraries(sonar_localization ${OpenCV_LIBS})

rosbuild_add_executable(sonar_wall_markers
  src/wall_markers.cpp
  src/util.cpp
)
