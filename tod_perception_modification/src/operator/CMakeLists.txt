find_package(glfw3 3.2 REQUIRED)

set(NODE_NAME OperatorPerceptionModification)
set(TARGETS ${TARGETS} ${NODE_NAME})
add_executable(${NODE_NAME}
    OperatorPerceptionModificationNode.cpp
    OperatorPerceptionModification.cpp
    PercModStatus.cpp)
add_dependencies(${NODE_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_include_directories(${NODE_NAME}
    PRIVATE
        ${catkin_INCLUDE_DIRS}
        ../helper)
target_link_libraries(${NODE_NAME} ${catkin_LIBRARIES})


set(NODE_NAME OperatorPopupNewScenario)
set(TARGETS ${TARGETS} ${NODE_NAME})
find_package(Qt5 REQUIRED COMPONENTS Widgets Core Network)
set(CMAKE_AUTOMOC ON) # needed for QOBJECT to be linked correctly
set(CMAKE_AUTOUIC ON) # needed for QOBJECT to be linked correctly
add_executable(${NODE_NAME}
    PopupNewScenarioNode.cpp
    PopupNewScenario.cpp
)
add_dependencies(${NODE_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}
)
target_include_directories(${NODE_NAME}
    PUBLIC
    ${catkin_INCLUDE_DIRS}
    ${CMAKE_CURRENT_BINARY_DIR}
)
target_link_libraries(${NODE_NAME}
    PUBLIC
    ${catkin_LIBRARIES}
    Qt5::Widgets
    Qt5::Core
    PahoMqttCpp::paho-mqttpp3
)
target_compile_features(${NODE_NAME} PUBLIC cxx_std_17)

install(TARGETS ${TARGETS}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
