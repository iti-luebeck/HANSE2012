set(CMAKE_TOOLCHAIN_FILE cmake/toolchain-avr-gcc.cmake)

cmake_minimum_required(VERSION 2.8)

project(hanse_atmega CXX C)

set(SRCS
  src/main.cpp
  src/serial.cpp
  src/i2ccomm.cpp
  src/error.cpp
  src/watchdog.cpp
)

add_definitions(-std=c++11)
add_definitions(-DF_CPU=16000000 -Wall)

set(HANSE_ATMEGA_MCUS atmega644p CACHE STRING foo FORCE)
set(HANSE_ATMEGA_UPLOAD_MCU atmega644p CACHE STRING foo FORCE)

add_avr_executable(hanse_atmega ${SRCS})

