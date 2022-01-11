cmake_minimum_required(VERSION 2.8.3)
project(flex_sensor)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  serial
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    serial
    std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_definitions(-std=c++11 -Wall -Wextra)


############
# BINARIES #
############
add_executable(flex_reader
        src/flex_reader.cpp
        )
target_link_libraries(flex_reader
        ${catkin_LIBRARIES}
      )
