#include "pch.hpp"

namespace mrover{
	class GPSLinearization : public rclcpp::Node {

		private:


		// callbacks
		void gps_callback(const sensor_msgs::msg::NavSatFix &gps_msg, const mrover::msg::FixStatus &gps_status_message);	
		
		// publishers and subscribers
        message_filters::Subscriber<sensor_msgs::msg::NavSatFix> gps_sub;
		message_filters::Subscriber<mrover::msg::FixStatus> gps_status_sub;

		rclcpp::Publisher<sensor_msgs::msg::Vector3Stamped>::SharedPtr pos_pub;

		// synchronizers
        std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime
            <sensor_msgs::msg::NavSatFix, mrover::msg::FixStatus>>> gps_and_status_sync;
	}; // class GPSLinearization
} // namespace mrover