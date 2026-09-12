#pragma once

// C++ Standard Library Headers, std namespace
#include <memory>
#include <functional>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>
#include <cmath>

// OpenCV Headers, cv namespace
#include <opencv2/aruco.hpp>
#include <opencv2/core/mat.hpp>

// ROS Headers, ros namespace
# include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <mrover/msg/starter_project_tag.hpp>

namespace mrover {
    class Localization : public rclcpp::Node {
    private:
        rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::ConstSharedPtr gpsSubscriber;
		rclcpp::Subscription<sensor_msgs::msg::Imu>::ConstSharedPtr imuSubscriber;
		SE3d pose(R3d(0, 0, 0), SO3d::Identity());

        rclcpp::Publisher<msg::SE3>::SharedPtr posePublisher;


    public:
        Localization();

        /**
         * Called when we receive GPS data
         *
         * @param gps_msg
         */
        void gpsCallback(const sensor_msgs::msg::NavSatFix::ConstSharedPtr &gps_msg);

		/**
         * Called when we receive IMU data
         *
         * @param imu_msg
         */
        void imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr &imu_msg);

		/**
		This is a utility function that should convert spherical (latitude, longitude)
        coordinates into cartesian (x, y, z) coordinates using the specified reference point
        as the center of the tangent plane used for approximation.
        :param spherical_coord: the spherical coordinate to convert,
                                given as a vector [latitude, longitude]
        :param reference_coord: the reference coordinate to use for conversion,
                                given as a vector [latitude, longitude]
        :returns: the approximated cartesian coordinates in meters, given as a vector [x, y, z]
		 */
		static auto sphericalToCartesian(std::vector<double> spherical_coord, std::vector<double> reference_coord) -> std::vector<double> {
			double R = 6371000;
			double PI = 3.1415926535;
			double x = R * (spherical_coord[0] - reference_coord[0]);
			double y = R * (spherical_coord[1] - reference_coord[1]) * std::cos(reference_coord[0] * PI / 180.0);
			return {x, y, 0};
		}
    };
}