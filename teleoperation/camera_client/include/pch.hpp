#pragma once

#include <chrono>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

#include <QApplication>
#include <QCheckBox>
#include <QComboBox>
#include <QDockWidget>
#include <QFormLayout>
#include <QGridLayout>
#include <QImage>
#include <QLabel>
#include <QLineEdit>
#include <QMainWindow>
#include <QMediaPlayer>
#include <QPushButton>
#include <QScrollArea>
#include <QVBoxLayout>
#include <QVideoWidget>
#include <QWidget>

#include <cv_bridge/cv_bridge.h>

#include <mrover/srv/media_control.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <gst_utils.hpp>
