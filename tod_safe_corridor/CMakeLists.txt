cmake_minimum_required(VERSION 3.0.2)
set(PROJECT_NAME tod_safe_corridor)
project(${PROJECT_NAME})

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    tod_msgs
    tf2
    tf2_ros
    geometry_msgs
    tod_core
    tod_helper
    tod_network
    tod_safety_monitoring
)

catkin_package(
  LIBRARIES ${PROJECT_NAME}_lib
  INCLUDE_DIRS include
  CATKIN_DEPENDS tod_core tod_helper tod_msgs
)

add_subdirectory(src/common)

option(OPERATOR "Build operator targets" ON)
if (${OPERATOR})
  add_subdirectory(src/operator)
else()
    message("skipping ${PROJECT_NAME} operator targets")
endif()

option(VEHICLE "Build vehicle targets" ON)
if (${VEHICLE})
  add_subdirectory(src/vehicle)
else()
    message("skipping ${PROJECT_NAME} vehicle targets")
endif()

install(DIRECTORY
    launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
