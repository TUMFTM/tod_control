cmake_minimum_required(VERSION 3.0.2)
set(PROJECT_NAME tod_pure_pursuit)
project(${PROJECT_NAME})

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    tod_msgs
    tf2
    tf2_ros
    geometry_msgs
    message_generation
    tod_core
    nav_msgs
    tod_helper
)

add_message_files(
    DIRECTORY msg
    FILES ppLog.msg
    )
generate_messages(DEPENDENCIES std_msgs)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}_lib
    CATKIN_DEPENDS roscpp tod_msgs
)

option(VEHICLE "Build vehicle targets" ON)
if (${VEHICLE})
    add_subdirectory(src)
else()
    message("skipping ${PROJECT_NAME} vehicle targets")
endif()

install(DIRECTORY launch include
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
