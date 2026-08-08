#include <chrono>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include "motor/brushed.hpp"


namespace mrover {
    class RelayHWBridge : public rclcpp::Node {

    public:
        RelayHWBridge() : Node{"relay_hw_bridge"} {}

        auto init() -> void {
            mRelay = std::make_shared<BrushedController<Meters>>(shared_from_this(), "jetson", "relay_controller");
            
            mRelayService = create_service<std_srvs::srv::SetBool>(
                    "set_relay",
                    [this](std_srvs::srv::SetBool::Request::SharedPtr const& req, std_srvs::srv::SetBool::Response::SharedPtr const& res) -> void {
                        setRelayCallback(req, res);
                    }
                );
            
                
            mCommandTimer = create_wall_timer(
                        std::chrono::milliseconds{50},
                        [this]() -> void {
                            mRelay->setDesiredThrottle(Percent{mRelayOn ? 1.0 : 0.0});
                        });
        };


    private:
        bool mRelayOn = false;

        std::shared_ptr<BrushedController<Meters>> mRelay;

        rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr mRelayService;

        rclcpp::TimerBase::SharedPtr mCommandTimer;

        auto setRelayCallback(std_srvs::srv::SetBool::Request::SharedPtr const& req, std_srvs::srv::SetBool::Response::SharedPtr const& res) -> void {
            mRelayOn = req->data;

            res->success = true;
            res->message = mRelayOn ? "Relay turned ON" : "Relay turned OFF"; 
        }

        };
};// namespace mrover


auto main(int const argc, char** argv) -> int {
    rclcpp::init(argc, argv);
    auto const relayBridge = std::make_shared<mrover::RelayHWBridge>();
    relayBridge->init();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(relayBridge);
    executor.spin();

    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
