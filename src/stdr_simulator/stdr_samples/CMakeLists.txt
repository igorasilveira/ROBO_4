cmake_minimum_required(VERSION 2.8.3)
project(stdr_samples)

find_package(catkin REQUIRED COMPONENTS
  roslib
  roscpp
  tf
  stdr_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
)

set(CMAKE_BUILD_TYPE Release)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
  CATKIN_DEPENDS
    roslib
    roscpp
    tf
    stdr_msgs
    geometry_msgs
    sensor_msgs
    nav_msgs
)

add_executable(robo_mapping
  src/mapping/main.cpp
  src/mapping/mapping.cpp)
add_dependencies(robo_mapping ${stdr_msgs_EXPORTED_TARGETS}) # wait for stdr_msgs to be build
target_link_libraries(robo_mapping
  ${catkin_LIBRARIES}
)

# Install excecutables
install(TARGETS robo_mapping
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)