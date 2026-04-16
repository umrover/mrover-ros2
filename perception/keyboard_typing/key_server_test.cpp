#include "pch.hpp"
#include <chrono>
#include <memory>
#include <thread>

namespace mrover {
    class TypingDeltasServer : public rclcpp::Node {
    public:
        using TypingDeltas = mrover::action::TypingDeltas;
        using GoalHandleTypingDeltas = rclcpp_action::ServerGoalHandle<TypingDeltas>;

        explicit TypingDeltasServer(rclcpp::NodeOptions const& options = rclcpp::NodeOptions())
            : Node("typing_deltas_server", options) {
            using namespace std::placeholders;

            this->action_server_ = rclcpp_action::create_server<TypingDeltas>(
                    this,
                    "typing_deltas",
                    std::bind(&TypingDeltasServer::handle_goal, this, _1, _2),
                    std::bind(&TypingDeltasServer::handle_cancel, this, _1),
                    std::bind(&TypingDeltasServer::handle_accepted, this, _1));
        }

    private:
        rclcpp_action::Server<TypingDeltas>::SharedPtr action_server_;

        rclcpp_action::GoalResponse handle_goal(
                rclcpp_action::GoalUUID const& uuid,
                std::shared_ptr<TypingDeltas::Goal const> goal) {
            RCLCPP_INFO_STREAM(this->get_logger(), "Received goal request");
            (void) uuid;
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }

        rclcpp_action::CancelResponse handle_cancel(
                std::shared_ptr<GoalHandleTypingDeltas> const goal_handle) {
            RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
            (void) goal_handle;
            return rclcpp_action::CancelResponse::ACCEPT;
        }

        void handle_accepted(std::shared_ptr<GoalHandleTypingDeltas> const goal_handle) {
            using namespace std::placeholders;
            // this needs to return quickly to avoid blocking the executor, so spin up a new thread
            std::thread{std::bind(&TypingDeltasServer::execute, this, _1), goal_handle}.detach();
        }

        void execute(std::shared_ptr<GoalHandleTypingDeltas> const goal_handle) {
            auto const goal = goal_handle->get_goal();
            auto result = std::make_shared<TypingDeltas::Result>();
            auto feedback = std::make_shared<TypingDeltas::Feedback>();

            result->success = false;
            feedback->dist_remaining = 10;

            // Test by spinning for 10 seconds
            RCLCPP_INFO_STREAM(this->get_logger(), "Sleeping for 10 sec, with periodic feedback");

            for (int i = 0; i <= 10; i++) {
                if (goal_handle->is_canceling()) {
                    RCLCPP_INFO_STREAM(this->get_logger(), "Cancelling");
                    result->success = false;
                    goal_handle->canceled(result);
                    return;
                }

                feedback->dist_remaining -= 1;
                goal_handle->publish_feedback(feedback);
                std::this_thread::sleep_for(std::chrono::seconds{1});
            }

            RCLCPP_INFO_STREAM(this->get_logger(), "Waking up");
            result->success = true;
            goal_handle->succeed(result);
        }
    };
} // namespace mrover

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<mrover::TypingDeltasServer>());
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}