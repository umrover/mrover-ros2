cmake_minimum_required(VERSION 3.8)
project(mrover)

include(cmake/macros.cmake)

set(CMAKE_CXX_STANDARD 23)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(manif REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

file(GLOB_RECURSE MROVER_MESSAGE_FILE_PATHS RELATIVE ${CMAKE_CURRENT_LIST_DIR} CONFIGURE_DEPENDS msg/*.msg)
message(STATUS "Found message files: ${MROVER_MESSAGE_FILE_PATHS}")

file(GLOB_RECURSE MROVER_SERVICE_FILE_PATHS RELATIVE ${CMAKE_CURRENT_LIST_DIR} CONFIGURE_DEPENDS srv/*.srv)
message(STATUS "Found service files: ${MROVER_SERVICE_FILE_PATHS}")

rosidl_generate_interfaces(${PROJECT_NAME}
  ${MROVER_MESSAGE_FILE_PATHS}
  ${MROVER_SERVICE_FILE_PATHS}
  DEPENDENCIES sensor_msgs geometry_msgs
)

ament_python_install_package(geometry)
ament_python_install_package(navigation)
ament_python_install_package(state_machine)

install(PROGRAMS
  navigation/nav.py
  navigation/stuck_detector/stuck_detector.py
  localization/rover_gps_driver.py
  localization/gps_linearization.py
  localization/basestation_gps_driver.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
