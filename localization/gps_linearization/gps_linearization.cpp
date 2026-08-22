#include "gps_linearization.hpp"
/*
Converts geodetic GPS coordinates (latitude, longitude, altitude) to local ENU coordinates (x, y, z) using a reference.
These are spherical and cartesian coordinate systems, respectively.
The former would be hard when working on autonomous tasks.
The latter allows us to use the TF tree and simplifies math.

This approximation is valid for small distances around the reference point.
At competition, it should be moved from the Wilson Center to MDRS, or better yet the starting point of the course.

*/
/*
def __init__(self) -> None:
        super().__init__("gps_linearization")

        self.ref_lat = self.get_parameter("ref_lat").value
        self.ref_lon = self.get_parameter("ref_lon").value
        self.ref_alt = self.get_parameter("ref_alt").value
        self.world_frame = self.get_parameter("world_frame").value

        self.pos_pub = self.create_publisher(Vector3Stamped, "linearized_position", 10)

        self.gps_sub = Subscriber(self, NavSatFix, "/gps/fix")
        self.gps_status_sub = Subscriber(self, FixStatus, "/gps_fix_status")

        self.synchronizer = TimeSynchronizer([self.gps_sub, self.gps_status_sub], 10)
        self.synchronizer.registerCallback(self.gps_callback)

    def gps_callback(self, gps_msg: NavSatFix, gps_status_msg: FixStatus):
        if gps_status_msg.fix_type.fix == FixType.NO_SOL:
            self.get_logger().warn("Received invalid GPS data, ignoring")
            return

        x, y, _ = geodetic2enu(
            gps_msg.latitude, gps_msg.longitude, 0.0, self.ref_lat, self.ref_lon, self.ref_alt, deg=True
        )
        self.pos_pub.publish(Vector3Stamped(header=gps_msg.header, vector=Vector3(x=x, y=y)))


def main() -> None:
    try:
        rclpy.init(args=sys.argv)
        rclpy.spin(GPSLinearization())
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        pass
*/

namespace mrover {
	GPSLinearization::GPSLinearization(boost::asio::io_context& io) : Node("rover_gps_driver"), serial(io) { 
		// connect to serial
		std::vector<ParameterWrapper> params{
			{"world_frame", world_frame, std::string("map")},
			{"ref_lat", ref_lat, 42.293195},
			{"ref_lon", ref_lon, -83.7096706},
			{"ref_alt", ref_alt, 0.0}
		};

		ParameterWrapper::declareParameters(this, params);

		// serial.open(port);
		// std::string port_string = std::to_string(baud);
		// serial.set_option(boost::asio::serial_port_base::baud_rate(baud));

		// RCLCPP_INFO(get_logger(), "Connected to GPS via serial!");

		// // publishers and subscribers
        
		pos_pub = this->create_publisher<sensor_msgs::msg::Vector3Stamped>("linearized_position", 10);
		
        gps_sub.subscribe(this, "/gps/fix");
        gps_status_sub.subscribe(this, "/gps_fix_status");
		// gps_sub = this->create_subscription<sensor_msgs::msg::NavSatFix>("/gps/fix", 10, [&](rtcm_msgs::msg::Message::ConstSharedPtr const& rtcm_message) {
		// 	gps_callback(rtcm_message);
		// });
        // synchronizers
        uint32_t queue_size = 10;

        gps_and_status_sync = std::make_shared<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::NavSatFix, mrover::msg::FixStatus>>>(
            message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::NavSatFix, mrover::msg::FixStatus>(queue_size),
            gps_sub,
            gps_status_sub
        );

        gps_and_status_sync->setAgePenalty(0.5);
        gps_and_status_sync->registerCallback(&GPSLinearization::gps_callback, this);

    }
	void GPSLinearization::gps_callback(const sensor_msgs::msg::NavSatFix &gps_msg, const mrover::msg::FixStatus &gps_status_msg) {
		if (gps_status_msg.fix_type.fix == FixType.NO_SOL):
            RCLCPP_WARN(get_logger(), "Received invalid GPS data, ignoring");
            return
        
        // TODO figure out how to include cppmap3d and use the equivalent function from their library; this code is from pythonmap3d.
        // x, y, _ = geodetic2enu(
        //     gps_msg.latitude, gps_msg.longitude, 0.0, self.ref_lat, self.ref_lon, self.ref_alt, deg=True
        // )
        int x = gps_msg.latitude; // TODO remove, just here so it can run
        int x = gps_msg.longitude;
        self.pos_pub.publish(Vector3Stamped(header=gps_msg.header, vector=Vector3(x=x, y=y)))

        pos_pub->publish(Vector3Stamped(gps_msg.header, Vector3(x,y,0)));
        
	}
}