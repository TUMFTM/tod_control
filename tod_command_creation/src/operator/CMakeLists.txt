set(TARGETS ${TARGETS} OperatorCommandCreator)
add_executable(OperatorCommandCreator CommandCreatorNode.cpp CommandCreator.cpp)
add_dependencies(OperatorCommandCreator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(OperatorCommandCreator ${catkin_LIBRARIES})
target_include_directories(OperatorCommandCreator PRIVATE ${catkin_INCLUDE_DIRS})

set(TARGETS ${TARGETS} OperatorPrimaryCommandSender)
add_executable(OperatorPrimaryCommandSender PrimaryCommandSender.cpp)
add_dependencies(OperatorPrimaryCommandSender ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(OperatorPrimaryCommandSender ${catkin_LIBRARIES})
target_include_directories(OperatorPrimaryCommandSender PRIVATE ${catkin_INCLUDE_DIRS})

set(TARGETS ${TARGETS} OperatorSecondaryCommandSender)
add_executable(OperatorSecondaryCommandSender SecondaryCommandSender.cpp)
add_dependencies(OperatorSecondaryCommandSender ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(OperatorSecondaryCommandSender ${catkin_LIBRARIES})
target_include_directories(OperatorSecondaryCommandSender PRIVATE ${catkin_INCLUDE_DIRS})

install(TARGETS ${TARGETS}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
