#include "click_ik.hpp"
#include "mrover/msg/detail/arm_status__struct.hpp"
#include <geometry_msgs/msg/detail/vector3__struct.hpp>
#include <rclcpp/create_timer.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp_action/server.hpp>
#include <sensor_msgs/msg/detail/point_cloud2__struct.hpp>
#include <tf2/exceptions.h>


namespace mrover {

    ClickIkNode::ClickIkNode(rclcpp::NodeOptions const& options) : Node("click_ik", options) {

        server = rclcpp_action::create_server<action::ClickIk>(
            this,
            "click_ik",
            std::bind(&ClickIkNode::startClickIk, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&ClickIkNode::handle_cancel, this),
            std::bind(&ClickIkNode::handle_accepted, this)
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

        RCLCPP_INFO(get_logger(), "Starting action server");
        server.registerGoalCallback([this]() { startClickIk(); });
        server.registerPreemptCallback([this] { cancelClickIk(); });
        server.start();
        RCLCPP_INFO(get_logger(), "Action server started");
    }
    
    rclcpp_action::CancelResponse ClickIkNode::handle_cancel(){

    };
        
    void handle_accepted(){

    };

    auto ClickIkNode::startClickIk(const rclcpp_action::GoalUUID& uuid, action::ClickIk_Goal::ConstSharedPtr goal) -> rclcpp_action::GoalResponse {
        RCLCPP_INFO(get_logger(),"Executing goal");
        auto target_point = spiralSearchInImg(static_cast<size_t>(goal->point_in_image_x), static_cast<size_t>(goal->point_in_image_y));
        
        if(!target_point.has_value()) {
            RCLCPP_WARN(get_logger(), "Target point does not exist.");
            return rclcpp_action::GoalResponse();// fix
        }

        geometry_msgs::msg::Vector3 pose;

        double offset = 0.1; // make sure we don't collide by moving back a little from the target
        pose.set__x(target_point.value().x - offset);
        pose.set__y(target_point.value().y);
        pose.set__z(target_point.value().z);

        
        
        message.pos = pose;
        // message.target.header.frame_id = "zed_left_camera_frame";
        timer = mNh.createTimer(ros::Duration(0.010), [&, pose](ros::TimerEvent const) {
            if (server.isPreemptRequested()) {
                timer.stop();
                return;
            }
            // Check if done
            float const tolerance = 0.02; // 2 cm
            try {
                SE3d arm_position = SE3Conversions::fromTfTree(mTfBuffer, "arm_e_link", "zed_left_camera_frame");
                double distance = pow(pow(arm_position.x() + ArmController::END_EFFECTOR_LENGTH - pose.position.x, 2) + pow(arm_position.y() - pose.position.y, 2) + pow(arm_position.z() - pose.position.z, 2), 0.5);
                RCLCPP_INFO("Distance to target: %f", distance);
                mrover::ClickIkFeedback feedback;
                feedback.distance = static_cast<float>(distance);
                server.publishFeedback(feedback);

                if (distance < tolerance) {
                    timer.stop();
                    mrover::ClickIkResult result;
                    result.success = true;
                    server.setSucceeded(result);
                    return;
                }
                // Otherwise publish message
                mIkPub.publish(message);
            } catch (tf2::ExtrapolationException& e) {
                ROS_WARN("ExtrapolationException (due to lag?): %s", e.what());
            }
        });
        
    }

    void ClickIkNode::startClickIk() {
        // RCLCPP_INFO("Executing goal");

        // mrover::ClickIkGoalConstPtr const& goal = server.acceptNewGoal();
        // auto target_point = spiralSearchInImg(static_cast<size_t>(goal->pointInImageX), static_cast<size_t>(goal->pointInImageY));

        //Check if optional has value
        if (!target_point.has_value()) {
            //Handle gracefully
            RCLCPP_WARN(get_logger(),"Target point does not exist.");
            return;
        }

        geometry_msgs::Pose pose;

        double offset = 0.1; // make sure we don't collide by moving back a little from the target
        pose.position.x = target_point.value().x - offset;
        pose.position.y = target_point.value().y;
        pose.position.z = target_point.value().z;

        message.target.pose = pose;
        message.target.header.frame_id = "zed_left_camera_frame";
        timer = mNh.createTimer(ros::Duration(0.010), [&, pose](ros::TimerEvent const) {
            if (server.isPreemptRequested()) {
                timer.stop();
                return;
            }
            // Check if done
            float const tolerance = 0.02; // 2 cm
            try {
                SE3d arm_position = SE3Conversions::fromTfTree(mTfBuffer, "arm_e_link", "zed_left_camera_frame");
                double distance = pow(pow(arm_position.x() + ArmController::END_EFFECTOR_LENGTH - pose.position.x, 2) + pow(arm_position.y() - pose.position.y, 2) + pow(arm_position.z() - pose.position.z, 2), 0.5);
                RCLCPP_INFO("Distance to target: %f", distance);
                mrover::ClickIkFeedback feedback;
                feedback.distance = static_cast<float>(distance);
                server.publishFeedback(feedback);

                if (distance < tolerance) {
                    timer.stop();
                    mrover::ClickIkResult result;
                    result.success = true;
                    server.setSucceeded(result);
                    return;
                }
                // Otherwise publish message
                mIkPub.publish(message);
            } catch (tf2::ExtrapolationException& e) {
                ROS_WARN("ExtrapolationException (due to lag?): %s", e.what());
            }
        });
    }

    void ClickIkNode::cancelClickIk() {
        timer.stop();
    }

    void ClickIkNode::pointCloudCallback(sensor_msgs::PointCloud2ConstPtr const& msg) {
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
                NODELET_WARN("Tag center point not finite: [%f %f %f]", point.x, point.y, point.z);

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

    auto ClickIkNode::statusCallback(ArmStatus const& status) -> void {
        if (!status.status) {
            cancelClickIk();
            if (server.isActive()) {
                mrover::ClickIkResult result;
                result.success = false;
                server.setAborted(result, "Arm position unreachable");
                NODELET_WARN("Arm position unreachable");
            }
        }
    }

} // namespace mrover

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(mrover::ClickIkNode);