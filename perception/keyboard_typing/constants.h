#ifndef KEYBOARD_TYPING_CONSTANTS_H
#define KEYBOARD_TYPING_CONSTANTS_H

#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/aruco/dictionary.hpp>
#include <string>

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

#endif // KEYBOARD_TYPING_CONSTANTS_H