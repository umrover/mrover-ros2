cmake_minimum_required(VERSION 3.16)

project(camera_client VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Qt5 5.15 REQUIRED COMPONENTS Widgets Multimedia MultimediaWidgets)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

add_executable(camera_client
    include/Gst.hpp
    include/GstRtpVideoCreatorWidget.hpp
    include/GstVideoGridWidget.hpp
    include/VideoSelectorWidget.hpp
    include/MainWindow.hpp
    src/MainWindow.cpp
    src/main.cpp
)

target_include_directories(camera_client PRIVATE include)

target_link_libraries(camera_client PRIVATE Qt::Widgets Qt::Multimedia Qt::MultimediaWidgets)

set_target_properties(camera_client PROPERTIES
    WIN32_EXECUTABLE ON
    MACOSX_BUNDLE ON
)
