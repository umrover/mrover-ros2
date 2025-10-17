cmake_minimum_required(VERSION 3.25)
project(mrover VERSION 2025.0.0 LANGUAGES C CXX)

# Supress ROS CMake warning about Python.
cmake_policy(SET CMP0148 OLD)

set(CMAKE_CXX_STANDARD 23)
set(CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_RPATH "${CMAKE_BINARY_DIR}")

# Generate compile_commands.json for clangd language server.
# Can be used by VSCode, CLion, VIM, etc.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(MROVER_CI "Enable CI options" OFF)
option(MROVER_CLANG_TIDY "Enable static analysis with Clang tidy" ${MROVER_CI})
option(MROVER_WARNINGS_AS_ERRORS "Treat warnings as errors" ${MROVER_CI})

set(node_plugins "")

if (APPLE)
    # Ensures that homebrew packages are never used over miniforge packages.
    # TODO (ali): Pretty sure this doesn't do anything on macs
    set(CMAKE_IGNORE_PATH /opt/homebrew)
    # Find mamba python
    find_package(PythonLibs REQUIRED)
    # link_libraries(${PYTHON_LIBRARIES})
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -Wno-missing-field-initializers -Wno-deprecated-declarations)

    if (MROVER_WARNINGS_AS_ERRORS)
        message(STATUS "Treating warnings as errors")
        add_compile_options(-Werror)
    endif ()
endif ()

# Ccache is a compiler cache.
# It speeds up recompilation by caching previous compilations and detecting when the same compilation is being done again.
# It is global, so even if you delete the build directory, it will still work.
# This can cause problems in rare cases so run "ccache -C" to clear the cache.
find_program(CCACHE_FOUND ccache)
if (CCACHE_FOUND)
    set(CMAKE_C_COMPILER_LAUNCHER ccache)
    set(CMAKE_CXX_COMPILER_LAUNCHER ccache)
    set(CMAKE_CUDA_COMPILER_LAUNCHER ccache)
endif ()


## Dependencies

find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rtcm_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(urdf REQUIRED)
find_package(std_srvs REQUIRED)
find_package(magic_enum REQUIRED)

find_package(Assimp NAMES Assimp assimp REQUIRED)
find_package(Bullet REQUIRED)
find_package(glfw3 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
if (NOT APPLE)
    find_package(TBB REQUIRED)
    link_libraries(TBB::tbb)
endif()

find_package(dawn QUIET)
if (dawn_FOUND)
    message(STATUS "Using Dawn system install")
else ()
    message(STATUS "Using Dawn from source")
    add_library(webgpu SHARED IMPORTED)

    set(WEBGPU_BUILD_DIR ${CMAKE_CURRENT_LIST_DIR}/deps/dawn/out/Release)
    if (APPLE)
        set(WEBGPU_SHARED_LIB ${WEBGPU_BUILD_DIR}/src/dawn/native/libwebgpu_dawn.dylib)
    else ()
        set(WEBGPU_SHARED_LIB ${WEBGPU_BUILD_DIR}/src/dawn/native/libwebgpu_dawn.so)
    endif ()
    if (EXISTS ${WEBGPU_SHARED_LIB})
        target_include_directories(webgpu INTERFACE ${CMAKE_CURRENT_LIST_DIR}/deps/dawn/include ${WEBGPU_BUILD_DIR}/gen/include)
        set_property(TARGET webgpu PROPERTY IMPORTED_LOCATION ${WEBGPU_SHARED_LIB})

        set(dawn_FOUND TRUE)
    else ()
        message(WARNING "Dawn not found. If on Ubuntu install with 'sudo apt install -f ./pkg/libdawn-dev.deb'. Or build from source with ./scripts/build_dawn.sh")
    endif ()
endif ()

find_package(MANIF::manif QUIET)
if (NOT manif_FOUND)
    if (EXISTS ${CMAKE_CURRENT_LIST_DIR}/deps/manif/include/manif)
        set(BUILD_TESTING_OLD ${BUILD_TESTING})
        set(BUILD_TESTING OFF)
        add_subdirectory(deps/manif EXCLUDE_FROM_ALL SYSTEM)
        set(BUILD_TESTING ${BUILD_TESTING_OLD})
        add_library(MANIF::manif ALIAS manif)

        set(manif_FOUND TRUE)
    else()
        message(FATAL_ERROR "Manif not found. If on Ubuntu install with 'sudo apt install -f ./pkg/libmanif-dev.deb'. Or build from source with 'git submodule update --init deps/manif' and make sure it is non-empty")
    endif()
endif()

find_package(PkgConfig REQUIRED)
pkg_search_module(NetLink libnl-3.0 IMPORTED_TARGET QUIET)
pkg_search_module(NetLinkRoute libnl-route-3.0 IMPORTED_TARGET QUIET)
pkg_search_module(Gst gstreamer-1.0 IMPORTED_TARGET QUIET)
pkg_search_module(GstApp gstreamer-app-1.0 IMPORTED_TARGET QUIET)
pkg_search_module(LibUdev libudev IMPORTED_TARGET QUIET)

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

include_directories(BEFORE SYSTEM preload)

include(cmake/macros.cmake)
# include(cmake/deps.cmake)

## Messages & Services & Actions

file(GLOB_RECURSE MROVER_MESSAGE_FILE_PATHS RELATIVE ${CMAKE_CURRENT_LIST_DIR} CONFIGURE_DEPENDS msg/*.msg)
message(STATUS "Found message files: ${MROVER_MESSAGE_FILE_PATHS}")

file(GLOB_RECURSE MROVER_SERVICE_FILE_PATHS RELATIVE ${CMAKE_CURRENT_LIST_DIR} CONFIGURE_DEPENDS srv/*.srv)
message(STATUS "Found service files: ${MROVER_SERVICE_FILE_PATHS}")

# Starter Project
message(STATUS "PROJ NAME: ${PROJECT_NAME}")

set(MROVER_CMAKE_INCLUDES
    starter_project/autonomy/AutonomyStarterProject.cmake
)

foreach (MROVER_CMAKE_INCLUDE ${MROVER_CMAKE_INCLUDES})
    include(${MROVER_CMAKE_INCLUDE})
endforeach ()

message(STATUS "Found message files: ${MROVER_MESSAGE_FILE_PATHS}")

rosidl_generate_interfaces(${PROJECT_NAME}
    ${MROVER_MESSAGE_FILE_PATHS}
    ${MROVER_SERVICE_FILE_PATHS}
    DEPENDENCIES nav_msgs sensor_msgs geometry_msgs
)

# Starter project 
# TODO (ali): delete once AutonomyStarterProject.cmake is fixed
mrover_add_node(starter_project_perception ${CMAKE_CURRENT_LIST_DIR}/starter_project/autonomy/src/*.cpp)
target_link_libraries(starter_project_perception ${OpenCV_LIBS})
ament_target_dependencies(starter_project_perception rclcpp std_msgs sensor_msgs)


# Install our executable so that ROS knows about it
# This allows us to launch it with ros2 run
install(PROGRAMS
    starter_project/autonomy/src/localization.py
    starter_project/autonomy/src/navigation/navigation_starter_project.py
    DESTINATION
    lib/${PROJECT_NAME}
)

install(DIRECTORY starter_project/autonomy/launch DESTINATION share/${PROJECT_NAME})

## Python

ament_python_install_package(navigation)
ament_python_install_package(lie)
ament_python_install_package(state_machine)

## C++

# Libraries

include_directories(perception)

mrover_add_library(lie lie/*.cpp lie)
target_link_libraries(lie MANIF::manif)
ament_target_dependencies(lie rclcpp geometry_msgs tf2 tf2_ros)

mrover_add_header_only_library(units units)
mrover_add_header_only_library(loop_profiler loop_profiler)
mrover_add_header_only_library(parameter_utils parameter_utils)
mrover_add_header_only_library(gst_utils gst_utils)
target_link_libraries(gst_utils INTERFACE magic_enum::magic_enum PkgConfig::Gst PkgConfig::GstApp)

# Simulator

mrover_add_node(simulator simulator/*.cpp simulator simulator/pch.hpp)
ament_target_dependencies(simulator rclcpp rclcpp_components nav_msgs sensor_msgs geometry_msgs tf2 tf2_ros urdf)
target_link_libraries(simulator
    assimp::assimp glfw3webgpu webgpu glfw webgpu_hpp imgui
    ${BULLET_LIBRARIES}
    lie Eigen3::Eigen
    opencv_core opencv_imgcodecs opencv_imgproc
    loop_profiler
)
target_include_directories(simulator SYSTEM PRIVATE ${BULLET_INCLUDE_DIRS} ${OPENCV_INCLUDE_DIRS})


# Localization

mrover_add_node(rover_gps_driver localization/rover_gps_driver.cpp)
ament_target_dependencies(rover_gps_driver rclcpp sensor_msgs rtcm_msgs)
target_link_libraries(rover_gps_driver parameter_utils)

mrover_add_node(heading_filter localization/heading_filter.cpp)
ament_target_dependencies(heading_filter rclcpp rcutils sensor_msgs builtin_interfaces message_filters)
target_link_libraries(heading_filter lie)

mrover_add_node(pose_filter localization/pose_filter/*.cpp)
ament_target_dependencies(pose_filter rclcpp tf2 tf2_ros geometry_msgs sensor_msgs)
target_link_libraries(pose_filter lie)

# Perception

find_package(ZED QUIET)
find_package(CUDA QUIET)
if(ZED_FOUND AND CUDA_FOUND)
    enable_language(CUDA)
    # CUDA Compile Options
    add_library(cuda_compiler_flags INTERFACE)
    target_compile_options(cuda_compiler_flags INTERFACE 
        -Wno-pedantic 
        -Wno-deprecated 
        -Wno-unused-parameter 
        -diag-suppress=815 
        -diag-suppress=780 
        -Wno-deprecated-copy 
        -Wno-unused-command-line-argument 
        -Wno-ignored-qualifiers 
        -Wno-sometimes-uninitialized
    )

    # ZED Wrapper

    set(THRUST_WRAPPED_NAMESPACE wrapped_thrust)
    set(CUB_WRAPPED_NAMESPACE wrapped_cub)

    add_definitions(-DTHRUST_WRAPPED_NAMESPACE=${THRUST_WRAPPED_NAMESPACE})
    add_definitions(-DCUB_WRAPPED_NAMESPACE=${CUB_WRAPPED_NAMESPACE})

    mrover_add_component(zed perception/zed_wrapper/*.c* perception/zed_wrapper/pch.hpp ZedWrapper)
    target_compile_options(zed_component PRIVATE $<$<COMPILE_LANGUAGE:CXX>:-std=c++20>)
    mrover_link_component(zed parameter_utils lie MANIF::manif ${CUDA_LIBRARIES} loop_profiler cuda_compiler_flags)
    mrover_ament_component(zed rclcpp rclcpp_components sensor_msgs ZED CUDA tf2 tf2_ros)

    mrover_executable_from_component(zed perception/executables/zed.cpp zed_component)

    # Learning Library
    # TODO(john): Update to use the new API
	mrover_add_library(tensorrt tensorrt/*.c* tensorrt)
	target_compile_options(tensorrt PRIVATE -Wno-deprecated-declarations $<$<COMPILE_LANGUAGE:CUDA>:-std=c++17>)
	target_include_directories(tensorrt PRIVATE ${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES})
	target_link_libraries(tensorrt opencv_core opencv_dnn opencv_imgproc lie nvinfer nvonnxparser tbb cuda_compiler_flags)

    # Object Detector
    mrover_add_component(object_detector perception/object_detector/*.c* perception/object_detector/pch.hpp StereoObjectDetector ImageObjectDetector)
    mrover_link_component(object_detector opencv_core opencv_dnn opencv_imgproc lie tbb tensorrt opencv_imgcodecs opencv_highgui loop_profiler parameter_utils cuda_compiler_flags)
    mrover_ament_component(object_detector rclcpp sensor_msgs rclcpp_components CUDA tf2 tf2_ros)

    mrover_executable_from_component(stereo_object_detector perception/executables/stereo_object_detector.cpp object_detector_component)
    mrover_executable_from_component(image_object_detector perception/executables/image_object_detector.cpp object_detector_component)
else()
    message("ZED not found...")
endif()

# Tag Detector
mrover_add_component(tag_detector perception/tag_detector/*.cpp perception/tag_detector/pch.hpp StereoTagDetector ImageTagDetector)
mrover_ament_component(tag_detector rclcpp rclcpp_components tf2 tf2_ros)
mrover_link_component(tag_detector lie opencv_core opencv_aruco opencv_imgproc loop_profiler parameter_utils)

mrover_executable_from_component(stereo_tag_detector perception/executables/stereo_tag_detector.cpp tag_detector_component)
mrover_executable_from_component(image_tag_detector perception/executables/image_tag_detector.cpp tag_detector_component)

# Cost Map
mrover_add_component(cost_map perception/cost_map/*.cpp perception/cost_map/pch.hpp CostMapNode)
mrover_ament_component(cost_map rclcpp rclcpp_components tf2 tf2_ros)
mrover_link_component(cost_map lie parameter_utils)

mrover_executable_from_component(cost_map perception/executables/cost_map.cpp cost_map_component)

# Embedded

mrover_add_header_only_library(messaging esw/messaging)

if (NOT APPLE)
    mrover_add_header_only_library(can_device esw/can_device)
    target_link_libraries(can_device INTERFACE ${rclcpp_LIBRARIES} moteus)

    mrover_add_library(motor_library esw/motor_library/*.cpp esw/motor_library)
    ament_target_dependencies(motor_library rclcpp std_srvs)
    target_link_libraries(motor_library can_device units messaging parameter_utils)

    macro(mrover_add_esw_motor_bridge_node name sources)
        mrover_add_node(${name} ${sources})
        ament_target_dependencies(${name} rclcpp std_srvs)
        target_link_libraries(${name} can_device units messaging)
        target_link_libraries(${name} motor_library)
    endmacro()

    mrover_add_esw_motor_bridge_node(drive_hw_bridge esw/drive_hw_bridge.cpp)
    mrover_add_esw_motor_bridge_node(arm_hw_bridge esw/arm_hw_bridge.cpp)
    mrover_add_esw_motor_bridge_node(mast_gimbal_hw_bridge esw/mast_gimbal_hw_bridge.cpp)

    mrover_add_node(pdlb_hw_bridge esw/pdlb_hw_bridge.cpp)
    ament_target_dependencies(pdlb_hw_bridge rclcpp std_srvs)
    target_link_libraries(pdlb_hw_bridge can_device units messaging)
endif()

mrover_add_node(differential_drive_controller esw/differential_drive_controller.cpp)
ament_target_dependencies(differential_drive_controller rclcpp std_srvs)
target_link_libraries(differential_drive_controller units messaging)

mrover_add_node(arm_controller teleoperation/arm_controller/arm_controller.cpp teleoperation/arm_controller/pch.hpp)
target_link_libraries(arm_controller lie)
ament_target_dependencies(arm_controller rclcpp)

mrover_add_node(led esw/led.cpp)
ament_target_dependencies(led rclcpp std_srvs)

if (Gst_FOUND AND GstApp_FOUND)
    mrover_add_node(usb_camera esw/usb_camera/*.cpp esw/usb_camera/pch.hpp)
    target_link_libraries(usb_camera PkgConfig::Gst PkgConfig::GstApp opencv_core opencv_imgcodecs parameter_utils)
    ament_target_dependencies(usb_camera rclcpp)

    if (LibUdev_FOUND)
        mrover_add_component(gst_camera_server esw/gst_camera_server/*.c* esw/gst_camera_server/pch.hpp GstCameraServer)
        mrover_link_component(gst_camera_server PkgConfig::Gst PkgConfig::GstApp PkgConfig::LibUdev opencv_core opencv_videoio opencv_imgcodecs opencv_imgproc gst_utils parameter_utils)
        mrover_ament_component(gst_camera_server rclcpp sensor_msgs rclcpp_components std_srvs)

        mrover_executable_from_component(gst_camera_server esw/gst_camera_server/main.cpp gst_camera_server_component)
    endif ()
endif ()

if (NetLink_FOUND AND NetLinkRoute_FOUND)
    mrover_add_node(can_bridge esw/can_bridge/*.cpp esw/can_bridge/pch.hpp)
    target_link_libraries(can_bridge PkgConfig::NetLink PkgConfig::NetLinkRoute)
    ament_target_dependencies(can_bridge rclcpp)
endif ()

# Teleop Qt5 Camera Client
find_package(Qt5 5.15 REQUIRED COMPONENTS Widgets Multimedia MultimediaWidgets)

mrover_add_node(camera_client teleoperation/camera_client/*.*pp teleoperation/camera_client/include/pch.hpp)
target_include_directories(camera_client PRIVATE teleoperation/camera_client/include)
target_link_libraries(camera_client gst_utils Qt::Widgets Qt::Multimedia Qt::MultimediaWidgets opencv_core opencv_imgcodecs)
ament_target_dependencies(camera_client rclcpp std_srvs sensor_msgs)
set_target_properties(camera_client PROPERTIES
    AUTOMOC ON
    AUTORCC ON
    AUTOUIC ON
)

## Install

# You must add Python scripts here to be able to "ros2 launch" them in terminal
install(PROGRAMS
    perception/panorama.py
    navigation/nav.py
    navigation/stuck_detector/stuck_detector.py
    localization/gps_linearization.py
    localization/basestation_gps_driver.py
    superstructure/superstructure.py
    scripts/debug_course_publisher.py
    scripts/debug_service.py
    scripts/astar_debug.py
    scripts/custom_costmap.py
    scripts/visualizer.py
    scripts/image_capture.py
    teleoperation/basestation_gui/gui_frontend.sh
    teleoperation/basestation_gui/gui_backend.sh
    teleoperation/basestation_gui/gui_chromium.sh

    # starter project sources
    starter_project/autonomy/src/localization.py
    starter_project/autonomy/src/navigation/navigation_starter_project.py
    DESTINATION
    lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})

ament_package()
