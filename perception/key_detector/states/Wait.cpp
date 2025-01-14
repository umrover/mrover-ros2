#include "Wait.hpp"
#include "key_detector/states/TargetKey.hpp"
#include <rclcpp/utilities.hpp>

namespace mrover {

	auto Wait::onLoop() -> State*{
		while(rclcpp::ok()){
            auto logger = rclcpp::get_logger("Wait");
            RCLCPP_INFO_STREAM(logger, "Entered Wait " <<  "\n");
            //sleepRate.sleep();
            RCLCPP_INFO_STREAM(logger, "Exiting Wait " <<  "\n");

			return new TargetKey();
		}
	}
}
