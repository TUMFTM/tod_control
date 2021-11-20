cmake_minimum_required(VERSION 2.8.3)
set(PROJECT_NAME tod_command_creation)
project(${PROJECT_NAME})

add_compile_options(-std=c++17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    tod_msgs
    tod_network
    tod_helper
)

catkin_package(
    CATKIN_DEPENDS roscpp tod_msgs tod_helper tod_network
)

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
