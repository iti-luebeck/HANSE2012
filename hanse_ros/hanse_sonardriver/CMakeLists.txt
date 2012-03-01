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

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

FIND_PACKAGE(Qt4 REQUIRED)
INCLUDE(${QT_USE_FILE})
ADD_DEFINITIONS(${QT_DEFINITIONS})

#add dynamic reconfigure api
rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)

QT4_WRAP_CPP(qextserialport_HEADERS_MOC
  qextserialport/qextserialenumerator.h
  qextserialport/qextserialport.h
  qextserialport/qextserialport_global.h
)

set(SONAR_SOURCES
  src/sonardatasourceserial.cpp
  src/sonarreturndata.cpp
  src/sonarswitchcommand.cpp
  src/scanningsonarswitchcommand.cpp
  src/echosounderswitchcommand.cpp

  qextserialport/qextserialport.cpp
  qextserialport/posix_qextserialport.cpp
  qextserialport/qextserialenumerator_unix.cpp

  ${qextserialport_HEADERS_MOC}
)

rosbuild_add_executable(scanningsonardriver
  src/scanningsonardriver.cpp
  ${SONAR_SOURCES}
)
target_link_libraries(scanningsonardriver ${QT_LIBRARIES})

rosbuild_add_executable(echosounderdriver
  src/echosounderdriver.cpp
  ${SONAR_SOURCES}
)
target_link_libraries(echosounderdriver ${QT_LIBRARIES})


rosbuild_add_executable(sonarvisualize
  src/sonarvisualize.cpp
)

rosbuild_add_executable(recordeddatapublisher
  src/recordeddatapublisher.cpp
  src/sonarreturndata.cpp
  src/sonarswitchcommand.cpp
  src/scanningsonarswitchcommand.cpp
)
target_link_libraries(recordeddatapublisher ${QT_LIBRARIES})

include_directories(qextserialport)
