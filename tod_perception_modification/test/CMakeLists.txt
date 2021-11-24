set(NODE_NAME TestPercModStatus)
add_executable(${NODE_NAME}
    testPercModStatus.cpp
    ../src/operator/PercModStatus.cpp)
add_dependencies(${NODE_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${NODE_NAME}
    ${catkin_LIBRARIES}
    ${GTEST_LIBRARIES})
target_include_directories(${NODE_NAME} PUBLIC
    ${catkin_INCLUDE_DIRS}
    ../src
    ../src/operator
    ../src/helper)


set(TARGET_CHECKER testObjectList)
set(CHECKER_SRC
    testObjectList.cpp)
add_executable(${TARGET_CHECKER} 
    ${CHECKER_SRC}
    )
target_include_directories(${TARGET_CHECKER} PRIVATE
    ${catkin_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}
    ../src/helper)
add_dependencies(${TARGET_CHECKER} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${TARGET_CHECKER}
    PRIVATE
    ${catkin_LIBRARIES}
    ${catkin_LIBRARIES}
    ${GTEST_LIBRARIES})
set_property(TARGET ${TARGET_CHECKER} PROPERTY CXX_STANDARD 17)


