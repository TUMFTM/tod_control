set(TARGETS ${TARGETS} SafeTrajCreator)
add_executable(SafeTrajCreator SafeTrajCreatorNode.cpp SafeTrajCreator.cpp)
add_dependencies(SafeTrajCreator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(SafeTrajCreator ${catkin_LIBRARIES} ${PROJECT_NAME}_lib)
target_include_directories(SafeTrajCreator PRIVATE ${catkin_INCLUDE_DIRS})
target_compile_features(SafeTrajCreator PUBLIC cxx_std_17)

set(TARGETS ${TARGETS} OperatorTrajectorySender)
add_executable(OperatorTrajectorySender OperatorTrajectorySender.cpp)
target_link_libraries(OperatorTrajectorySender ${catkin_LIBRARIES})
target_include_directories(OperatorTrajectorySender PRIVATE ${catkin_INCLUDE_DIRS})
target_compile_features(OperatorTrajectorySender PUBLIC cxx_std_17)

set(TARGETS ${TARGETS} OperatorGateStateReceiver)
add_executable(OperatorGateStateReceiver OperatorGateStateReceiver.cpp)
target_link_libraries(OperatorGateStateReceiver ${catkin_LIBRARIES})
target_include_directories(OperatorGateStateReceiver PRIVATE ${catkin_INCLUDE_DIRS})
target_compile_features(OperatorGateStateReceiver PUBLIC cxx_std_17)

install(TARGETS ${TARGETS}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
