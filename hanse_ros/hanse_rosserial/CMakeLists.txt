cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

set(ARDUINO_SDK_PATH /home/forouher/Arduino/build/linux/work)

rosbuild_find_ros_package(rosserial_arduino)
include(${rosserial_arduino_PACKAGE_PATH}/cmake_scripts/rosserial.cmake)
#====================================================================#
#  Chatter                                                           #
#====================================================================#

set(FIRMWARE_NAME hako_rosserial)

set(${FIRMWARE_NAME}_BOARD atmega644)         # Arduino Target board
set(${FIRMWARE_NAME}_SRCS src/main.cpp
			  src/Logger.cpp
			  src/arduino_lib/Time.cpp
			  src/arduino_lib/TimeAlarms.cpp
			  src/arduino_lib/Wire.cpp
			  src/arduino_lib/twi.c
			)
set(${FIRMWARE_NAME}_PORT /dev/ttyUSB_jtag1)  # Serial upload port
generate_ros_firmware(${FIRMWARE_NAME})
