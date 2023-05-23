if (${VEHICLE})
    set(TARGETS ${TARGETS} FblBasedTrackingController)
    add_executable(FblBasedTrackingController
        FblBasedTrackingControllerNode.cpp
        FblBasedTrackingController.cpp
        FeedbackLinearization.cpp
        ../common/ControllerBase.cpp
        ../common/CommonFeedback.cpp
        ../common/TrajectoryHandler.cpp
        )
    add_dependencies(FblBasedTrackingController ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    target_link_libraries(FblBasedTrackingController ${catkin_LIBRARIES})
    target_include_directories(FblBasedTrackingController PRIVATE ${catkin_INCLUDE_DIRS} ../common/)
else()
    message("skipping ${PROJECT_NAME} vehicle fbl targets")
endif()

install(TARGETS ${TARGETS}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
