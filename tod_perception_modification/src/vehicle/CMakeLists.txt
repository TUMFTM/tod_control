set(NODE VehiclePerceptionModification)
set(TARGETS ${TARGETS} ${NODE})
add_executable(${NODE}
    VehiclePerceptionModificationNode.cpp
    VehiclePerceptionModification.cpp)
add_dependencies(${NODE} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${NODE} ${catkin_LIBRARIES})
target_include_directories(${NODE}
    PUBLIC
        ${catkin_INCLUDE_DIRS}
        ../helper)


set(NODE VehicleFakeObjectsPublisher)
set(TARGETS ${TARGETS} ${NODE})
add_executable(${NODE}
    FakeObjectsPublisherNode.cpp
    FakeObjectsPublisher.cpp
    Timer.cpp)
add_dependencies(${NODE} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${NODE} ${catkin_LIBRARIES})
target_include_directories(${NODE}
    PUBLIC
        ${catkin_INCLUDE_DIRS})


set(NODE VehicleFakeCellsPublisher)
set(TARGETS ${TARGETS} ${NODE})
add_executable(${NODE}
    FakeCellsPublisherNode.cpp
    FakeCellsPublisher.cpp  
    Timer.cpp)
add_dependencies(${NODE} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${NODE} ${catkin_LIBRARIES})
target_include_directories(${NODE}
    PUBLIC
        ${catkin_INCLUDE_DIRS})

        
set(NODE VehicleRouteToPath)
set(TARGETS ${TARGETS} ${NODE})
add_executable(${NODE}
    RouteToPathNode.cpp
    RouteToPath.cpp
    )
add_dependencies(${NODE} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${NODE} ${catkin_LIBRARIES})
target_include_directories(${NODE}
    PUBLIC
        ${catkin_INCLUDE_DIRS}
        ../helper)


set(NODE VehicleControlCommandCreator)
set(TARGETS ${TARGETS} ${NODE})
add_executable(${NODE}
    ControlCommandCreatorNode.cpp
    ControlCommandCreator.cpp
    )
add_dependencies(${NODE} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${NODE} ${catkin_LIBRARIES})
target_include_directories(${NODE}
    PUBLIC
        ${catkin_INCLUDE_DIRS})



set(NODE VehiclePathToCollisionFreePath)
set(TARGETS ${TARGETS} ${NODE})
add_executable(${NODE}
    PathToCollisionFreePathNode.cpp
    PathToCollisionFreePath.cpp
    )
add_dependencies(${NODE} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${NODE} ${catkin_LIBRARIES})
target_include_directories(${NODE}
    PUBLIC
        ${catkin_INCLUDE_DIRS}
        ../helper)



set(NODE VehicleCommunicator)
set(TARGETS ${TARGETS} ${NODE})
add_executable(${NODE}
    VehicleCommunicatorNode.cpp
    VehicleCommunicator.cpp)
add_dependencies(${NODE} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${NODE} ${catkin_LIBRARIES})
target_include_directories(${NODE}
    PUBLIC
        ${catkin_INCLUDE_DIRS}
        ../helper)


set(NODE VehiclePopupTrigger)
set(TARGETS ${TARGETS} ${NODE})
add_executable(${NODE}
    VehiclePopupTriggerNode.cpp)
add_dependencies(${NODE} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_include_directories(${NODE}
    PUBLIC
    ${catkin_INCLUDE_DIRS})
target_link_libraries(${NODE}
    ${catkin_LIBRARIES}
)

install(TARGETS ${TARGETS}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
