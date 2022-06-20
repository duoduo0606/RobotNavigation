cmake_minimum_required(VERSION 2.8.9)

PROJECT(RobotNavigation)

set( CMAKE_BUILD_TYPE "Debug" )
set( CMAKE_CXX_FLAGS_DEBUG "-O0 -Wall -g -ggdb" )
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/build)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories(${PROJECT_SOURCE_DIR}/include )
include_directories(/usr/local/include/eigen3/unsupported)
include_directories(/usr/local/include/qpOASES)
include_directories(/usr/local/include)
include_directories(/usr/include)

link_directories(/usr/local/lib)

AUX_SOURCE_DIRECTORY(./src DIR_SRCS)

ADD_EXECUTABLE(RobotNavigation ${DIR_SRCS})
target_link_libraries(RobotNavigation libqpOASES.a pthread i2c wiringPi)