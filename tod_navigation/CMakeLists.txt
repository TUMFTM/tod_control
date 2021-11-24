cmake_minimum_required(VERSION 3.10)
set(PROJECT_NAME tod_navigation)
project(${PROJECT_NAME})

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin REQUIRED COMPONENTS roscpp roslib nav_msgs tod_helper tod_msgs tod_network)
catkin_package(
    CATKIN_DEPENDS roscpp roslib nav_msgs tod_helper tod_msgs tod_network
)

option(VEHICLE "Build vehicle targets" ON)
if (${VEHICLE})
    set(TARGETS ${TARGETS} RouteProvider)
    add_executable(RouteProvider src/VehicleRouteProviderNode.cpp src/XmlRouteParser.cpp)
    add_dependencies(RouteProvider ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    target_include_directories(RouteProvider PRIVATE ${catkin_INCLUDE_DIRS})
    target_link_libraries(RouteProvider ${catkin_LIBRARIES} tinyxml2)
    target_compile_features(RouteProvider PUBLIC cxx_std_17)

    set(TARGETS ${TARGETS} VehicleRouteSender)
    add_executable(VehicleRouteSender src/VehicleRouteSender.cpp)
    target_include_directories(VehicleRouteSender PRIVATE ${catkin_INCLUDE_DIRS})
    target_link_libraries(VehicleRouteSender ${catkin_LIBRARIES})
    target_compile_features(VehicleRouteSender PUBLIC cxx_std_17)
else()
    message("skipping ${PROJECT_NAME} vehicle targets")
endif()

option(OPERATOR "Build operator targets" ON)
if (${OPERATOR})
    set(TARGETS ${TARGETS} OperatorRouteReceiver)
    add_executable(OperatorRouteReceiver src/OperatorRouteReceiver.cpp)
    target_include_directories(OperatorRouteReceiver PRIVATE ${catkin_INCLUDE_DIRS})
    target_link_libraries(OperatorRouteReceiver ${catkin_LIBRARIES})
    target_compile_features(OperatorRouteReceiver PUBLIC cxx_std_17)

    set(NODE OperatorRouteToMeshMsg)
    set(TARGETS ${TARGETS} ${NODE})
    add_executable(${NODE} src/OperatorRouteToMeshMsgNode.cpp)
    target_include_directories(${NODE} PRIVATE ${catkin_INCLUDE_DIRS})
    target_link_libraries(${NODE} ${catkin_LIBRARIES})
    target_compile_features(${NODE} PUBLIC cxx_std_17)
else()
    message("skipping ${PROJECT_NAME} operator targets")
endif()

option(TEST "Build test targets" OFF)
if (${TEST})
    add_subdirectory(test/)
else()
    # message("skipping ${PROJECT_NAME} test targets")
endif()

install(TARGETS ${TARGETS}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY 
    launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
