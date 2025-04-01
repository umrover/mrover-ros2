#include "lander_analysis.hpp"

namespace mrover {
    LanderAnalysis::LanderAnalysis(rclcpp::NodeOptions const& options) : rclcpp::Node{NODE_NAME}{
        RCLCPP_INFO_STREAM(get_logger(), "Created Node");
    }
};

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(mrover::LanderAnalysis);
