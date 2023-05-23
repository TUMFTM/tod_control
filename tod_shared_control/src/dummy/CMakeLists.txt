if (${VEHICLE})
    set(TARGETS ${TARGETS} TrajectoryGenerator)
    add_executable(TrajectoryGenerator TrajectoryGeneratorNode.cpp)
    target_link_libraries(TrajectoryGenerator ${catkin_LIBRARIES})
    target_include_directories(TrajectoryGenerator PRIVATE ${catkin_INCLUDE_DIRS})

    set(TARGETS ${TARGETS} ObjectListPublisher)
    add_executable(ObjectListPublisher ObjectListPublisher.cpp)
    target_link_libraries(ObjectListPublisher ${catkin_LIBRARIES})
    target_include_directories(ObjectListPublisher PRIVATE ${catkin_INCLUDE_DIRS})
else()
    message("skipping ${PROJECT_NAME} vehicle dummy targets")
endif()

install(TARGETS ${TARGETS}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
