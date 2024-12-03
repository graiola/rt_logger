cmake_minimum_required(VERSION 3.5)
project(rt_logger)

include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++17" COMPILER_SUPPORTS_CXX17)
check_cxx_compiler_flag("-std=c++14" COMPILER_SUPPORTS_CXX14)
check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_CXX11)
check_cxx_compiler_flag("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX17)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -Wmaybe-uninitialized -Wuninitialized")
elseif(COMPILER_SUPPORTS_CXX14)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wmaybe-uninitialized -Wuninitialized")
elseif(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wmaybe-uninitialized -Wuninitialized")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wmaybe-uninitialized -Wuninitialized")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 nor C++14 support. Please use a different C++ compiler.")
endif()

find_package(catkin QUIET)
find_package(ament_cmake QUIET)

if(${catkin_FOUND})
    set(ROS_VERSION "ros")
    add_definitions(-DROS)
    message(STATUS "Add ROS definition")

    find_package(message_generation REQUIRED)
    find_package(realtime_tools REQUIRED)
    find_package(std_msgs REQUIRED)

    include_directories(
        include
        ${realtime_tools_INCLUDE_DIRS}
        ${std_msgs_INCLUDE_DIRS}
    )

    add_message_files(
        DIRECTORY
            msg/ros
        FILES
            LoggerNumeric.msg
            LoggerNumericArray.msg
    )

    generate_messages(
        DEPENDENCIES
        std_msgs)

    catkin_package(
      INCLUDE_DIRS include
      CATKIN_DEPENDS realtime_tools std_msgs message_generation
    )

elseif(${ament_cmake_FOUND})
    set(ROS_VERSION "ros2")
    add_definitions(-DROS2)
    message(STATUS "Add ROS2 definition")

    find_package(rosidl_default_generators REQUIRED)
    find_package(realtime_tools REQUIRED)
    find_package(builtin_interfaces REQUIRED)

    include_directories(
        include
        ${realtime_tools_INCLUDE_DIRS}
        ${builtin_interfaces_INCLUDE_DIRS}
    )

    rosidl_generate_interfaces(${PROJECT_NAME}
        "msg/${ROS_VERSION}/LoggerNumeric.msg"
        "msg/${ROS_VERSION}/LoggerNumericArray.msg"
        DEPENDENCIES builtin_interfaces std_msgs
        #LIBRARY_NAME ${PROJECT_NAME}
    )

    ament_export_include_directories(include)
    ament_export_dependencies(realtime_tools builtin_interfaces rosidl_default_runtime std_msgs)

    ament_package()

endif()

install(DIRECTORY include/${PROJECT_NAME}
        DESTINATION ${CMAKE_INSTALL_PREFIX}/include)

#if (CATKIN_ENABLE_TESTING)
#    find_package(catkin REQUIRED COMPONENTS roscpp rostest realtime_tools std_msgs)
#    include_directories(include ${catkin_INCLUDE_DIRS})
#
#    add_rostest_gtest(rt_logger_test
#        test/rt_logger_test.test
#        test/rt_logger_test.cpp)
#    target_link_libraries(rt_logger_test ${catkin_LIBRARIES})
#endif()
