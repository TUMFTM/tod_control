set(TARGETS ${TARGETS} VehicleTrajectoryReceiver)
add_executable(VehicleTrajectoryReceiver VehicleTrajectoryReceiver.cpp)
target_link_libraries(VehicleTrajectoryReceiver ${catkin_LIBRARIES})
target_include_directories(VehicleTrajectoryReceiver PRIVATE ${catkin_INCLUDE_DIRS})
target_compile_features(VehicleTrajectoryReceiver PUBLIC cxx_std_17)

set(TARGETS ${TARGETS} VehicleGateStateSender)
add_executable(VehicleGateStateSender VehicleGateStateSender.cpp)
target_link_libraries(VehicleGateStateSender ${catkin_LIBRARIES})
target_include_directories(VehicleGateStateSender PRIVATE ${catkin_INCLUDE_DIRS})
target_compile_features(VehicleGateStateSender PUBLIC cxx_std_17)

install(TARGETS ${TARGETS}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)