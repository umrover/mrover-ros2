#pragma once

#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <cstdint>
#include <random>
#include <cmath>
#include <execution>
#include <sensor_msgs/distortion_models.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

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

	inline auto createNoisyPointCloud(sensor_msgs::msg::PointCloud2::UniquePtr const& msg) -> sensor_msgs::msg::PointCloud2::UniquePtr {
		// Copy the PointCloud Message
		sensor_msgs::msg::PointCloud2::UniquePtr ptr = std::make_unique<sensor_msgs::msg::PointCloud2>(*msg);
        std::random_device rd;
        std::mt19937 genPose(rd());
        std::normal_distribution<float> dPose(0, 0.01); // mean 0 m, std dev. 0.1 m

        std::mt19937 genNorm(rd());
        std::normal_distribution<float> dNorm(0, 0.01); // mean 0 m, std dev. 0.01

        auto * pointPtr = reinterpret_cast<Point*>(ptr->data.data());
        std::for_each(pointPtr, pointPtr + msg->height * msg->width, [&](Point& point) {
            std::size_t i = &point - pointPtr;
            Point& p = pointPtr[i];
            
            // add noise to xyz loc 
            p.x += dPose(genPose);
            p.y += dPose(genPose);
            p.z += dPose(genPose);

            // add noise to normal vectors
            // this will work execpt for when the normals are perfectly
            // in either the x or y direction, but close enough
            p.normal_x += dNorm(genNorm);
            p.normal_y += dNorm(genNorm);

            float sigZ = (p.normal_z < 0) ? -1.0f : 1.0f;
            p.normal_z = sigZ * std::sqrt(1 - p.normal_x * p.normal_x + p.normal_y * p.normal_y);
        });

		return ptr;
	}
} // namespace mrover
