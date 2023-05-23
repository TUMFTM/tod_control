if (${VEHICLE})
    set(TARGETS ${TARGETS} VehicleSenders)
    add_executable(VehicleSenders VehicleSendersNode.cpp)
    add_dependencies(VehicleSenders ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    target_link_libraries(VehicleSenders ${catkin_LIBRARIES})
    target_include_directories(VehicleSenders PRIVATE ${catkin_INCLUDE_DIRS})
else()
    message("skipping ${PROJECT_NAME} vehicle network targets")
endif()

if (${OPERATOR})
    set(TARGETS ${TARGETS} OperatorReceivers)
    add_executable(OperatorReceivers OperatorReceiversNode.cpp)
    add_dependencies(OperatorReceivers ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    target_link_libraries(OperatorReceivers ${catkin_LIBRARIES})
    target_include_directories(OperatorReceivers PRIVATE ${catkin_INCLUDE_DIRS})
else()
    message("skipping ${PROJECT_NAME} operator network targets")
endif()

install(TARGETS ${TARGETS}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
