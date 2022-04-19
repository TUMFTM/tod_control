add_library(${PROJECT_NAME}_lib 
    ClothoidPlanner.cpp
    LateralPlanner.cpp
    SweptPathCreator.cpp
    ConstantDeceleration.cpp)

target_link_libraries(${PROJECT_NAME}_lib PUBLIC ${catkin_LIBRARIES})

target_include_directories(${PROJECT_NAME}_lib
    PUBLIC
        ../../include/
    PRIVATE
        ../../include/tod_safe_corridor/
        ${catkin_INCLUDE_DIRS}
)
target_compile_features(${PROJECT_NAME}_lib PUBLIC cxx_std_17)

install(DIRECTORY ../../include/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}_lib 
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)