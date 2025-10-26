#include "click_ik.hpp"
#include "mrover/action/detail/click_ik__struct.hpp"
#include "mrover/msg/detail/arm_status__struct.hpp"
#include <chrono>
#include <cstddef>
#include <geometry_msgs/msg/detail/vector3__struct.hpp>
#include <memory>
#include <rclcpp/create_timer.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp_action/server.hpp>
#include <rclcpp_action/server_goal_handle.hpp>
#include <sensor_msgs/msg/detail/point_cloud2__struct.hpp>
#include <tf2/exceptions.h>
#include <tf2/exceptions.hpp>
#include <thread>


namespace mrover {

    ClickIkNode::ClickIkNode(rclcpp::NodeOptions const& options) : Node("click_ik", options) {

        server = rclcpp_action::create_server<action::ClickIk>(
            this,
            "click_ik",
            std::bind(&ClickIkNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&ClickIkNode::handle_cancel, this, std::placeholders::_1),
            std::bind(&ClickIkNode::handle_accepted, this, std::placeholders::_1)
        );

        mPcSub = create_subscription<sensor_msgs::msg::PointCloud2>("camera/left/points", 1, [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr const& msg) {
            pointCloudCallback(msg);
        });
        
        // IK Publisher
        mIkPub = create_publisher<msg::IK>("/arm_ik", 1);

        // ArmStatus subscriber
        mStatusSub = create_subscription<msg::ArmStatus>("arm_status", 1, [this](msg::ArmStatus const& msg) {
            statusCallback(msg);
        });

        mCurrentGoalHandle = nullptr;
    }
    
    rclcpp_action::GoalResponse ClickIkNode::handle_goal(const rclcpp_action::GoalUUID& uuid, action::ClickIk_Goal::ConstSharedPtr goal){
        RCLCPP_INFO(this->get_logger(), "Click Ik request received for point: (%d, %d)", goal->point_in_image_x, goal->point_in_image_y);
        (void) uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse ClickIkNode::handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<action::ClickIk>> goal_handle){
        RCLCPP_INFO(this->get_logger(), "Cancelling ClickIk Action.");
        (void) goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    };
        
    void ClickIkNode::handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<action::ClickIk>> goal_handle){
        if(mCurrentGoalHandle){
            action::ClickIk::Result::SharedPtr result_msg;
            mCurrentGoalHandle->canceled(result_msg);
        }
        mCurrentGoalHandle = goal_handle;
        std::thread{std::bind(&ClickIkNode::executeClickIk, this, std::placeholders::_1), goal_handle}.detach();
    };

    void ClickIkNode::executeClickIk(std::shared_ptr<rclcpp_action::ServerGoalHandle<action::ClickIk>> goal_handle) {
        RCLCPP_INFO(get_logger(),"Executing goal");
        if (!goal_handle) {
            RCLCPP_WARN(get_logger(), "Invalid ClickIK goal handle");
            return;
        }
        
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<action::ClickIk::Feedback>();
        auto result = std::make_shared<action::ClickIk::Result>();
        auto target_point = spiralSearchInImg(static_cast<size_t>(goal->point_in_image_x), static_cast<size_t>(goal->point_in_image_y));
        
        if(!target_point.has_value()) {
            RCLCPP_WARN(get_logger(), "Target point does not exist.");
            result->success = false;
            goal_handle->canceled(result);
            return; // fix
        }

        geometry_msgs::msg::Vector3 pose;

        double offset = 0.1; // make sure we don't collide by moving back a little from the target
        pose.set__x(target_point.value().x - offset);
        pose.set__y(target_point.value().y);
        pose.set__z(target_point.value().z);     
        

        message.pos = pose;
        // message.target.header.frame_id = "zed_left_camera_frame";
        timer = this->create_wall_timer(std::chrono::milliseconds(10), [&, pose](){
            if(goal_handle->is_canceling()){
                timer->cancel();
                return;
            }

            float const tolerance = 0.02;
            try {
                SE3d arm_position = SE3Conversions::fromTfTree(*mTfBuffer, "arm_e_link", "zed_left_camera_frame");
                double distance = pow(pow(arm_position.x() + ArmController::END_EFFECTOR_LENGTH - pose.x, 2) + pow(arm_position.y() - pose.y, 2) + pow(arm_position.z() - pose.z, 2), 0.5);
                RCLCPP_INFO(this->get_logger(), "Distance to target: %f", distance);
                feedback->distance = static_cast<float>(distance);
                goal_handle->publish_feedback(feedback);

                if (distance < tolerance) {
                    timer->cancel();
                    result->success = true;
                    goal_handle->succeed(result);
                    return;
                }
            } catch (tf2::ExtrapolationException& e) {
                RCLCPP_WARN(this->get_logger(), "ExtrapolationException (due to lag?): %s", e.what());
            }
            
        });        
    }


    void ClickIkNode::cancelClickIk() {
        timer->cancel();
    }

    void ClickIkNode::pointCloudCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr const& msg) {
        // Update current pointer to pointcloud data
        mPoints = reinterpret_cast<Point const*>(msg->data.data());
        mNumPoints = msg->width * msg->height;
        mPointCloudWidth = msg->width;
        mPointCloudHeight = msg->height;
    }

    auto ClickIkNode::spiralSearchInImg(size_t xCenter, size_t yCenter) -> std::optional<Point> {
        std::size_t currX = xCenter;
        std::size_t currY = yCenter;
        std::size_t radius = 0;
        int t = 0;
        constexpr int numPts = 16;
        bool isPointInvalid = true;
        Point point{};

        // Find the smaller of the two box dimensions so we know the max spiral radius
        std::size_t smallDim = std::min(mPointCloudWidth / 2, mPointCloudHeight / 2);

        while (isPointInvalid) {
            // This is the parametric equation to spiral around the center pnt
            currX = static_cast<size_t>(static_cast<double>(xCenter) + std::cos(t * 1.0 / numPts * 2 * M_PI) * static_cast<double>(radius));
            currY = static_cast<size_t>(static_cast<double>(yCenter) + std::sin(t * 1.0 / numPts * 2 * M_PI) * static_cast<double>(radius));

            // Grab the point from the pntCloud and determine if its a finite pnt
            point = mPoints[currX + currY * mPointCloudWidth];

            isPointInvalid = !std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z);
            if (isPointInvalid)
                RCLCPP_WARN(this->get_logger(), "Tag center point not finite: [%f %f %f]", point.x, point.y, point.z);

            // After a full circle increase the radius
            if (t % numPts == 0) {
                radius++;
            }

            // Increase the parameter
            t++;

            // If we reach the edge of the box we stop spiraling
            if (radius >= smallDim) {
                return std::nullopt;
            }
        }

        return std::make_optional<Point>(point);
    }

    auto ClickIkNode::statusCallback(msg::ArmStatus const& msg) -> void {
        if (!msg.status) {
            cancelClickIk();
            if (mCurrentGoalHandle) {
                auto result = std::make_shared<action::ClickIk::Result>();
                result->success = false;
                mCurrentGoalHandle->canceled(result);
                mCurrentGoalHandle = nullptr;
                RCLCPP_WARN(get_logger(), "Arm position unreachable");
            }
        }
    }

} // namespace mrover

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(mrover::ClickIkNode);