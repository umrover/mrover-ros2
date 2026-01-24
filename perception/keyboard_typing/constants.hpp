#ifndef KEYBOARD_TYPING_CONSTANTS_H
#define KEYBOARD_TYPING_CONSTANTS_H

#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/aruco/dictionary.hpp>
#include <string>
#include "pch.hpp"

bool logPose = false;

//constants used across keyboard typing

//------------------------------
int const SQUARES_VERTICALLY = 7;
int const SQUARES_HORIZONTALLY = 5;
float const SQUARE_LENGTH = 0.032998833333;
float const MARKER_LENGTH = 0.02;
int const LENGTH_M = 1056; // total length of the page in pixels
int const MARGIN_M = 37;   // size of the margin in pixels
std::string const SAVE_NAME = "ChArUco_Marker1.png";

int const DEFAULT_CAM_GAIN = 0;
int const DEFAULT_CAM_EXPOSURE = 156;
int const DEFAULT_CAM_BRIGHTNESS = 0;

auto const ARUCO_DICT = cv::aruco::DICT_4X4_50;
auto const DICTIONARY = cv::aruco::getPredefinedDictionary(ARUCO_DICT);
auto const BOARD = cv::aruco::CharucoBoard::create(
        SQUARES_VERTICALLY, SQUARES_HORIZONTALLY,
        SQUARE_LENGTH,
        MARKER_LENGTH,
        DICTIONARY);
auto const PARAMS = cv::aruco::DetectorParameters();
// auto const DETECTOR = cv::aruco::ArucoDetector(DICTIONARY, PARAMS);
// ------------------------------

double key_length = 0.01905;
//initial x coordinate of row
double secondRowX = -0.0095; 
double thirdRowX = -0.01345;
double fourthRowX = -0.009525;
//initial z coordinate of row
double secondRowZ = 0.0007112;
double thirdRowZ = 0.0017272;
double fourthRowZ = 0.0048514;
std::unordered_map<std::string, cv::Vec3d> keyboard = {
        {"z", cv::Vec3d{0,0,0}},
        {"x", cv::Vec3d{key_length,0,0}},
        {"c", cv::Vec3d{2*key_length,0,0}},
        {"v", cv::Vec3d{3*key_length,0,0}},
        {"b", cv::Vec3d{4*key_length,0,0}},
        {"n", cv::Vec3d{5*key_length,0,0}},
        {"m", cv::Vec3d{6*key_length,0,0}},

        {"a", cv::Vec3d{secondRowX, key_length,secondRowZ}},
        {"s", cv::Vec3d{secondRowX + key_length, key_length,secondRowZ}},
        {"d", cv::Vec3d{secondRowX + 2*key_length, key_length,secondRowZ}},
        {"f", cv::Vec3d{secondRowX + 3*key_length, key_length,secondRowZ}},
        {"g", cv::Vec3d{secondRowX + 4*key_length, key_length,secondRowZ}},
        {"h", cv::Vec3d{secondRowX + 5*key_length, key_length,secondRowZ}},
        {"j", cv::Vec3d{secondRowX + 6*key_length, key_length,secondRowZ}},
        {"k", cv::Vec3d{secondRowX + 7*key_length, key_length,secondRowZ}},
        {"l", cv::Vec3d{secondRowX + 8*key_length, key_length,secondRowZ}},

        {"q", cv::Vec3d{thirdRowX, 2*key_length,thirdRowZ}},
        {"w", cv::Vec3d{thirdRowX + key_length, 2*key_length,thirdRowZ}},
        {"e", cv::Vec3d{thirdRowX + 2*key_length, 2*key_length,thirdRowZ}},
        {"r", cv::Vec3d{thirdRowX + 3*key_length, 2*key_length,thirdRowZ}},
        {"t", cv::Vec3d{thirdRowX + 4*key_length, 2*key_length,thirdRowZ}},
        {"y", cv::Vec3d{thirdRowX + 5*key_length, 2*key_length,thirdRowZ}},
        {"u", cv::Vec3d{thirdRowX + 6*key_length, 2*key_length,thirdRowZ}},
        {"i", cv::Vec3d{thirdRowX + 7*key_length, 2*key_length,thirdRowZ}},
        {"o", cv::Vec3d{thirdRowX + 8*key_length, 2*key_length,thirdRowZ}},
        {"p", cv::Vec3d{thirdRowX + 9*key_length, 2*key_length,thirdRowZ}},

        {"backspace", cv::Vec3d{fourthRowX + 11*key_length + 0.028575, 3*key_length,fourthRowZ}}
};

// our coordinates
// +X is right
// +Y is up
// +Z is forward

// arm_fk coordinates
// -Y is right
// +Z is up
// +X is forwards

// y -> -x
// z -> y
// x -> z
Eigen::Vector3d zKeyTransformation = {0.06495,0.041575,0.0303784};

Eigen::Vector3d zKeyTransformation_new = {0.0303784,-0.06495,0.041575};

#endif // KEYBOARD_TYPING_CONSTANTS_H