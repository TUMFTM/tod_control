cmake_minimum_required(VERSION 3.0.2)
set(PROJECT_NAME tod_shared_control)
project(${PROJECT_NAME})

set(DEPRECATION_FLAG "-Wno-deprecated-declarations -Wno-format")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${DEPRECATION_FLAG}")
set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    message_generation
    nav_msgs
    visualization_msgs
    tf2_ros
    tod_msgs
    tod_network
    tod_helper
    tod_core
    )

add_message_files(
    DIRECTORY msg
    FILES MpcLog.msg SvcLog.msg FblLog.msg FloatArray.msg
    )

generate_messages(DEPENDENCIES std_msgs nav_msgs visualization_msgs tod_msgs visualization_msgs)

catkin_package(
    INCLUDE_DIRS # include
    # LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS roscpp message_generation tf2_ros nav_msgs tod_msgs tod_network tod_helper tod_core
    # DEPENDS system_lib
    )

option(OPERATOR "Build operator targets" ON)
option(VEHICLE "Build vehicle targets" ON)
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules" ${CMAKE_MODULE_PATH})
add_subdirectory(src)

# INSTALL
install(DIRECTORY
    launch
    config
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    )
