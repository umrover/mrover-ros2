## Starter Project
# CMake is a build scripting system
# It is a program whose sole purpose is to generate Makefiles (or Ninja build files)
# It is used extensively in industry
project(mrover VERSION 2025.0.0 LANGUAGES C CXX)

file(GLOB_RECURSE STARTER_PROJ_MESSAGE_PATHS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} CONFIGURE_DEPENDS 
        ${CMAKE_CURRENT_LIST_DIR}/msg/*.msg
)

list(APPEND MROVER_MESSAGE_FILE_PATHS ${STARTER_PROJ_MESSAGE_PATHS})

# TODO (ali): fix include cmake build
# Collect all cpp files in the src subdirectory to be used for perception
# file(GLOB_RECURSE STARTER_PROJECT_PERCEPTION_SOURCES "${CMAKE_CURRENT_LIST_DIR}/src/*.cpp")
# # Define a new CMake target, specifically a built C++ executable, that uses the found source files
# add_executable(starter_project_perception ${STARTER_PROJECT_PERCEPTION_SOURCES})
# # # Add custom msg type support
# rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
# # Link needed libraries
# target_link_libraries(starter_project_perception ${catkin_LIBRARIES} ${OpenCV_LIBS} ${cpp_typesupport_target})
# # Include needed directories
# target_include_directories(starter_project_perception PUBLIC ${catkin_INCLUDE_DIRS})
# #Include necessary package dependencies
# ament_target_dependencies(starter_project_perception rclcpp std_msgs sensor_msgs)


# Install our executable so that ROS knows about it
# This allows us to launch it with ros2 run
install(PROGRAMS
        starter_project/autonomy/src/localization.py
        starter_project/autonomy/src/navigation/navigation_starter_project.py
        DESTINATION
        lib/${PROJECT_NAME}
)