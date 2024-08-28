## Starter Project
# CMake is a build scripting system
# It is a program whose sole purpose is to generate Makefiles (or Ninja build files)
# It is used extensively in industry

# TODO: add your new message file here by replacing the *
file(GLOB_RECURSE MROVER_STARTER_PROJECT_MSG_PATHS RELATIVE ${CMAKE_SOURCE_DIR} CONFIGURE_DEPENDS msg/*)

# Collect all cpp files in the src subdirectory to be used for perception
file(GLOB_RECURSE STARTER_PROJECT_PERCEPTION_SOURCES "${CMAKE_CURRENT_LIST_DIR}/src/*.cpp")
# Define a new CMake target, specifically a built C++ executable, that uses the found source files
add_executable(starter_project_perception ${STARTER_PROJECT_PERCEPTION_SOURCES})
# Link needed libraries
target_link_libraries(starter_project_perception ${catkin_LIBRARIES} ${OpenCV_LIBS})
# Include needed directories
target_include_directories(starter_project_perception PUBLIC ${catkin_INCLUDE_DIRS})

ament_target_dependencies(starter_project_perception rclcpp std_msgs sensor_msgs)

# Install our executable so that ROS knows about it
# This allows us to launch it with ros2 run
install(PROGRAMS
        starter_project/autonomy/src/localization.py
        starter_project/autonomy/src/navigation/navigation_starter_project.py
        DESTINATION
        lib/${PROJECT_NAME}
)