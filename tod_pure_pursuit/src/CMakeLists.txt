add_library(${PROJECT_NAME}_lib PurePursuit.cpp)
add_dependencies(${PROJECT_NAME}_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_lib ${catkin_LIBRARIES})
target_include_directories(${PROJECT_NAME}_lib PRIVATE ${catkin_INCLUDE_DIRS} PUBLIC ../include)
target_compile_features(${PROJECT_NAME}_lib PRIVATE cxx_std_17)

add_executable(${PROJECT_NAME}_node PathTrackingControlNode.cpp PathTrackingControl.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${PROJECT_NAME}_lib)
target_include_directories(${PROJECT_NAME}_node PRIVATE ${catkin_INCLUDE_DIRS})
target_compile_features(${PROJECT_NAME}_node PRIVATE cxx_std_17)

install(TARGETS ${PROJECT_NAME}_node ${PROJECT_NAME}_lib
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
