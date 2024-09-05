#pragma once

#include <cstdint>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/distortion_models.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/image_encodings.hpp"

namespace mrover {

    /**
     * @brief Defines one element in the point cloud
     *
     * This definition HAS to match the below function which fills in the header.
     */
    struct Point {
        float x, y, z;
		std::uint8_t b, g, r, a;
        float normal_x, normal_y, normal_z;
        float curvature;
    } __attribute__((packed));

    inline void fillPointCloudMessageHeader(sensor_msgs::msg::PointCloud2::UniquePtr const& msg) {
        sensor_msgs::PointCloud2Modifier modifier{*msg};
        modifier.setPointCloud2Fields(
                8,
                "x", 1, sensor_msgs::msg::PointField::FLOAT32,
                "y", 1, sensor_msgs::msg::PointField::FLOAT32,
                "z", 1, sensor_msgs::msg::PointField::FLOAT32,
                "rgb", 1, sensor_msgs::msg::PointField::FLOAT32, // by convention rgb is stored as float32 even thought it is three bytes
                "normal_x", 1, sensor_msgs::msg::PointField::FLOAT32,
                "normal_y", 1, sensor_msgs::msg::PointField::FLOAT32,
                "normal_z", 1, sensor_msgs::msg::PointField::FLOAT32,
                "curvature", 1, sensor_msgs::msg::PointField::FLOAT32);
    }

} // namespace mrover
