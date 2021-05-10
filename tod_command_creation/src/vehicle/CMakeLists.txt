set(TARGETS ${TARGETS} VehiclePrimaryCommandReceiver)
add_executable(VehiclePrimaryCommandReceiver PrimaryCommandReceiver.cpp)
add_dependencies(VehiclePrimaryCommandReceiver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(VehiclePrimaryCommandReceiver ${catkin_LIBRARIES})
target_include_directories(VehiclePrimaryCommandReceiver PRIVATE ${catkin_INCLUDE_DIRS})

set(TARGETS ${TARGETS} VehicleSecondaryCommandReceiver)
add_executable(VehicleSecondaryCommandReceiver SecondaryCommandReceiver.cpp)
add_dependencies(VehicleSecondaryCommandReceiver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(VehicleSecondaryCommandReceiver ${catkin_LIBRARIES})
target_include_directories(VehicleSecondaryCommandReceiver PRIVATE ${catkin_INCLUDE_DIRS})

install(TARGETS ${TARGETS}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
