#include "localization.hpp"

// ROS Headers, ros namespace
#include <cmath>
#include <functional>
#include <iterator>
#include <limits>
#include <memory>
#include <numeric>
#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

auto main(int argc, char** argv) -> int {
    rclcpp::init(argc, argv);

    // "spin" blocks until our node dies
    rclcpp::spin(std::make_shared<mrover::Localization>());
    rclcpp::shutdown();

    return EXIT_SUCCESS;
}

namespace mrover {
	Localization::Localization() : Node("localization") {
        // Create subscribers for GPS and IMU data, linking them to our callback functions
		// Every time another node publishes to the /gps or /imu topics, it should call the respective callback
		// TODO
        gpsSubscriber = create_subscription<sensor_msgs::msg::NavSatFix>("/gps", 1, [this](sensor_msgs::msg::NavSatFix::ConstSharedPtr const& msg) {
            gpsCallback(msg);
        });
		imuSubscriber = create_subscription<sensor_msgs::msg::Imu>("/imu", 1, [this](sensor_msgs::msg::Imu::ConstSharedPtr const& msg) {
            imuCallback(msg);
        });

        // Create a publisher for our tag topic
        // See: http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29
        posePublisher = create_publisher<msg::StarterProjectTag>("localization", 1);

    }

    auto Localization::gpsCallback(const sensor_msgs::msg::NavSatFix::ConstSharedPtr &gps_msg) -> void {
        // reads the GPS location from the given NavSatFix message,
        // convert it to cartesian coordinates, store that value in `self.pose`, then publish
        // that pose to the TF tree.
		// TODO
		pose.position = R3d(sphericalToCartesian({gps_msg.latitude, gps_msg.longitude}));
		posePublisher->publish(pose);

    }

	auto Localization::imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr &imu_msg) -> void {
        // reads the orientation data from the given Imu message,
        // store that value in `self.pose`, then publish that pose to the TF tree.
		// TODO
		pose.orientation = SO3(imu_msg.orientation);
		posePublisher->publish(pose);

    }

} // namespace mrover