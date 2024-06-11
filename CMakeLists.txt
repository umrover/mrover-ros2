cmake_minimum_required(VERSION 3.25)
project(mrover VERSION 2025.0.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 23)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -Wno-missing-field-initializers)
endif ()

## Dependencies

find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(urdf REQUIRED)
find_package(std_srvs REQUIRED)

find_package(Assimp NAMES Assimp assimp REQUIRED)
find_package(Bullet REQUIRED)
find_package(glfw3 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(TBB REQUIRED)

find_package(manif QUIET)
find_package(dawn QUIET)

find_package(PkgConfig REQUIRED)
pkg_search_module(NetLink libnl-3.0 QUIET)
pkg_search_module(NetLinkRoute libnl-route-3.0 QUIET)

add_subdirectory(deps/imgui EXCLUDE_FROM_ALL SYSTEM)
add_subdirectory(deps/webgpuhpp EXCLUDE_FROM_ALL SYSTEM)
add_subdirectory(deps/glfw3webgpu EXCLUDE_FROM_ALL SYSTEM)
add_subdirectory(deps/mjbots EXCLUDE_FROM_ALL SYSTEM)

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # comment the line when a copyright and license is added to all source files
    set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # comment the line when this package is in a git repo and when
    # a copyright and license is added to all source files
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif ()

include(cmake/macros.cmake)

## Messages & Services & Actions

file(GLOB_RECURSE MROVER_MESSAGE_FILE_PATHS RELATIVE ${CMAKE_CURRENT_LIST_DIR} CONFIGURE_DEPENDS msg/*.msg)
message(STATUS "Found message files: ${MROVER_MESSAGE_FILE_PATHS}")

file(GLOB_RECURSE MROVER_SERVICE_FILE_PATHS RELATIVE ${CMAKE_CURRENT_LIST_DIR} CONFIGURE_DEPENDS srv/*.srv)
message(STATUS "Found service files: ${MROVER_SERVICE_FILE_PATHS}")

rosidl_generate_interfaces(${PROJECT_NAME}
        ${MROVER_MESSAGE_FILE_PATHS}
        ${MROVER_SERVICE_FILE_PATHS}
        DEPENDENCIES nav_msgs sensor_msgs geometry_msgs
)

## Python

ament_python_install_package(navigation)
ament_python_install_package(lie)
ament_python_install_package(state_machine)

## C++

include_directories(perception)

mrover_add_library(lie lie/*.cpp lie)
target_link_libraries(lie MANIF::manif)
ament_target_dependencies(lie rclcpp geometry_msgs tf2 tf2_ros)

mrover_add_header_only_library(units units)
mrover_add_header_only_library(loop_profiler loop_profiler)

mrover_add_node(simulator simulator/*.cpp simulator simulator/pch.hpp)
ament_target_dependencies(simulator rclcpp nav_msgs sensor_msgs geometry_msgs tf2 tf2_ros urdf)
rosidl_target_interfaces(simulator ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(simulator
        assimp::assimp glfw3webgpu webgpu glfw webgpu_hpp imgui
        ${BULLET_LIBRARIES}
        lie Eigen3::Eigen
        opencv_core opencv_imgcodecs opencv_imgproc
        loop_profiler
)
target_include_directories(simulator SYSTEM PRIVATE ${BULLET_INCLUDE_DIRS} ${OPENCV_INCLUDE_DIRS})

mrover_add_node(tag_detector perception/tag_detector/*.cpp perception/tag_detector/pch.hpp)
ament_target_dependencies(tag_detector rclcpp tf2 tf2_ros)
rosidl_target_interfaces(tag_detector ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(tag_detector lie opencv_core opencv_aruco opencv_imgproc loop_profiler TBB::tbb)

mrover_add_node(differential_drive_controller esw/differential_drive_controller.cpp)
target_link_libraries(differential_drive_controller units)
ament_target_dependencies(differential_drive_controller rclcpp)
rosidl_target_interfaces(differential_drive_controller ${PROJECT_NAME} rosidl_typesupport_cpp)

mrover_add_node(arm_controller teleoperation/arm_controller/arm_controller.cpp teleoperation/arm_controller/pch.hpp)
target_link_libraries(arm_controller lie)
ament_target_dependencies(arm_controller rclcpp)
rosidl_target_interfaces(arm_controller ${PROJECT_NAME} rosidl_typesupport_cpp)

if (NetLink_FOUND AND NetLinkRoute_FOUND)
    mrover_add_node(can_bridge esw/can_bridge/*.cpp esw/can_bridge/pch.hpp)
    target_link_libraries(can_bridge ${NetLink_LIBRARIES} ${NetLinkRoute_LIBRARIES})
    target_include_directories(can_bridge PRIVATE ${NetLink_INCLUDE_DIRS} ${NetLinkRoute_INCLUDE_DIRS})
    ament_target_dependencies(can_bridge rclcpp)
    rosidl_target_interfaces(can_bridge ${PROJECT_NAME} rosidl_typesupport_cpp)
endif ()

## Install

install(PROGRAMS
        navigation/nav.py
        navigation/stuck_detector/stuck_detector.py
        localization/rover_gps_driver.py
        localization/gps_linearization.py
        localization/basestation_gps_driver.py
        superstructure/superstructure.py
        scripts/debug_course_publisher.py
        DESTINATION
        lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME})

ament_package()
