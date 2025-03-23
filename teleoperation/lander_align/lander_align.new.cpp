#include "lander_align.new.hpp"

//LanderAlignActionServer member functions:
namespace mrover{

    using LanderAlign = action::LanderAlign;
    using GoalHandleLanderAlign = rclcpp_action::ServerGoalHandle<LanderAlign>;

    LanderAlignNode::LanderAlignNode(const rclcpp::NodeOptions& options) 
        : Node("lander_align", options) {
        //The constructor also instantiates a new action server
        action_server_ = rclcpp_action::create_server<LanderAlign>(
            this,
            "lander_align_action",
            std::bind(&LanderAlignNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&LanderAlignNode::handle_cancel, this, std::placeholders::_1),
            std::bind(&LanderAlignNode::handle_accepted, this, std::placeholders::_1));
    }

    //the callback for handling new goals (this implementation just accepts all goals)
    rclcpp_action::GoalResponse LanderAlignNode::handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const LanderAlign::Goal> goal) {
        RCLCPP_INFO(get_logger(), "Received goal request with order %d", goal->num);
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    //the callback for dealing with cancellation (this implementation just tells the client that it accepted the cancellation)
    rclcpp_action::CancelResponse LanderAlignNode::handle_cancel(const std::shared_ptr<GoalHandleLanderAlign> goal_handle) {
        RCLCPP_INFO(get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    //the callback for accepting a new goal and processing it
    void LanderAlignNode::handle_accepted(const std::shared_ptr<GoalHandleLanderAlign> goal_handle) {
        using namespace std::placeholders;
        // this needs to return quickly to avoid blocking the executor, so spin up a new thread
        std::thread{std::bind(&LanderAlignNode::execute, this, _1), goal_handle}.detach();
    }

    //all further processing and updates are done in the execute method in the new thread
    void LanderAlignNode::execute(const std::shared_ptr<GoalHandleLanderAlign> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Executing goal");
        auto result = std::make_shared<LanderAlign::Result>();
        // Check if goal is done
        if (rclcpp::ok()) {
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Goal succeeded");
        }
    }

    
}

int main(void) {
    return 0;
}