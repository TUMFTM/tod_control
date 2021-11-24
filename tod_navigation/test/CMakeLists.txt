set(EXECUTABLE RouteParserTest)
add_executable(${EXECUTABLE} testXmlParser.cpp
    ../src/XmlRouteParser.cpp
)
target_include_directories(${EXECUTABLE} PRIVATE
    ../src/
    ${catkin_INCLUDE_DIRS}
)
target_link_libraries(${EXECUTABLE} PRIVATE
    tinyxml2
    ${GTEST_LIBRARIES}
    ${catkin_LIBRARIES}
)
set_property(TARGET ${EXECUTABLE} PROPERTY CXX_STANDARD 17)
