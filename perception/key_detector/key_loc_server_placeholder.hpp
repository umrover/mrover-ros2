#include "pch.hpp"
#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class GetKeyLocServer : public rclcpp::Node {
public:
    GetKeyLocServer() : Node("get_key_loc_server") {
        service_ = this->create_service<mrover::srv::GetKeyLoc>(
                "GetKeyLoc",
                std::bind(&GetKeyLocServer::handle_request, this, _1, _2));

        RCLCPP_INFO(this->get_logger(), "Service 'GetKeyLoc' is ready.");
    }

private:
    void handle_request(
            std::shared_ptr<mrover::srv::GetKeyLoc::Request> const request,
            std::shared_ptr<mrover::srv::GetKeyLoc::Response> response) {
        RCLCPP_INFO(this->get_logger(), "Received request for key_name: %hhu", request->key);

        // Example logic: Assign coordinates based on the key name
        if (request->key == 'c') {
            response->x = 1.0;
            response->y = 2.0;
        } else if (request->key == 'd') {
            response->x = 4.0;
            response->y = 5.0;
        } else {
            response->x = 1.0;
            response->y = 2.0;
        }

        RCLCPP_INFO(this->get_logger(), "Responding with coordinates: x=%.2f, y=%.2f",
                    response->x, response->y);
    }

    rclcpp::Service<mrover::srv::GetKeyLoc>::SharedPtr service_;
};
