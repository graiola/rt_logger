cmake_minimum_required(VERSION 2.8.3)
project(rt_logger)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED
    roscpp realtime_tools std_msgs message_generation)

add_message_files(FILES
    LoggerNumeric.msg)

generate_messages(
    DEPENDENCIES
    std_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

if (CATKIN_ENABLE_TESTING)
    find_package(catkin REQUIRED COMPONENTS roscpp rostest realtime_tools std_msgs)
    include_directories(include ${catkin_INCLUDE_DIRS})

    add_rostest_gtest(rt_logger_test
        test/rt_logger_test.test
        test/rt_logger_test.cpp)
    target_link_libraries(rt_logger_test ${catkin_LIBRARIES})
endif()
