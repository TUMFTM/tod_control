cmake_minimum_required(VERSION 3.10)
set(PROJECT_NAME tod_perception_modification)
project(${PROJECT_NAME})


set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    tf2
    tf2_ros
    tod_msgs
    message_generation
    tod_network
    grid_map_core
    grid_map_ros
    grid_map_msgs
    tod_helper
    )
find_package(PahoMqttCpp REQUIRED)
find_package(Threads)

# ------------------------------------------------------------------------
# --------------------------- Generate Messages --------------------------
# ------------------------------------------------------------------------

set(MESSAGE_NAME perc_mod_msgs)
add_message_files(
    DIRECTORY msg
    FILES
    PercModOperatorApproval.msg
    PercModOperatorRequest.msg
    PercModVehicleResponse.msg
)
generate_messages(
    DEPENDENCIES
    std_msgs
    tod_msgs
    sensor_msgs
)
add_library(${MESSAGE_NAME} INTERFACE)

# ------------------------------------------------------------------------
# --------------------------- Build Executables --------------------------
# ------------------------------------------------------------------------

catkin_package(
        CATKIN_DEPENDS roscpp std_msgs tod_msgs tod_helper
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

add_subdirectory(src/network/)

# ------------------------------------------------------------------------
# --------------------------- Install launch file ------------------------
# ------------------------------------------------------------------------

install(DIRECTORY 
    launch
    yaml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# ------------------------------------------------------------------------
# --------------------------- Test Executables --------------------------
# ------------------------------------------------------------------------

option(TEST "Build test targets" OFF)
if (${TEST})
    add_subdirectory(test/)
else()
    # message("skipping ${PROJECT_NAME} test targets")
endif()

