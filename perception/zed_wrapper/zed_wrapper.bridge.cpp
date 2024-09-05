#include "zed_wrapper.hpp"

namespace mrover {

	constexpr static float DEG_TO_RAD = std::numbers::pi_v<float> / 180.0f;
    constexpr static std::uint64_t NS_PER_S = 1000000000;

	auto slTime2Ros(sl::Timestamp t) -> rclcpp::Time {
        return {static_cast<int32_t>(t.getNanoseconds() / NS_PER_S),
                static_cast<uint32_t>(t.getNanoseconds() % NS_PER_S)};
    }
};
