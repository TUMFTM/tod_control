catkin_add_gtest(${PROJECT_NAME}_test PurePursuitTest.cpp)
add_dependencies(${PROJECT_NAME}_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_test ${catkin_LIBRARIES} ${PROJECT_NAME}_lib)
target_include_directories(${PROJECT_NAME}_test PRIVATE ${catkin_INCLUDE_DIRS})
