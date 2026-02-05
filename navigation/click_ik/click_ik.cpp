#include "click_ik.hpp"
#include "lie.hpp"
#include "mrover/action/detail/click_ik__struct.hpp"
#include "mrover/action/detail/ik_image_sample__struct.hpp"
#include "mrover/msg/detail/arm_status__struct.hpp"
#include "mrover/msg/detail/ik__struct.hpp"
#include "mrover/srv/detail/ik_sample__struct.hpp"
#include <chrono>
#include <cstddef>
#include <future>
#include <geometry_msgs/msg/detail/vector3__struct.hpp>
#include <manif/impl/se3/SE3.h>
#include <memory>
#include <optional>
#include <queue>
#include <rclcpp/create_timer.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/future_return_code.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp_action/create_server.hpp>
#include <rclcpp_action/server.hpp>
#include <rclcpp_action/server_goal_handle.hpp>
#include <sensor_msgs/msg/detail/point_cloud2__struct.hpp>
#include <tf2/exceptions.h>
#include <tf2/exceptions.hpp>
#include <thread>


namespace mrover {

    ClickIkNode::ClickIkNode(rclcpp::NodeOptions const& options) : Node("click_ik", options) {

        auto handle_goal = [this](rclcpp_action::GoalUUID const& uuid, action::ClickIk_Goal::ConstSharedPtr const& goal) -> rclcpp_action::GoalResponse {
            RCLCPP_INFO(this->get_logger(), "Click Ik request received for point: (%d, %d)", goal->point_in_image_x, goal->point_in_image_y);
            (void) uuid;
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        };


        auto handle_cancel = [this](std::shared_ptr<rclcpp_action::ServerGoalHandle<action::ClickIk>> const& goal_handle) -> rclcpp_action::CancelResponse {
            RCLCPP_INFO(this->get_logger(), "Cancelling ClickIk Action.");
            (void) goal_handle;
            return rclcpp_action::CancelResponse::ACCEPT;
        };

        auto handle_accepted = [this](std::shared_ptr<rclcpp_action::ServerGoalHandle<action::ClickIk>> const& goal_handle) -> void {
            if (mCurrentGoalHandle)
                abortClickIk();
            mCurrentGoalHandle = goal_handle;
            auto thread_executor = [this, goal_handle]() {
                return this->executeClickIk(goal_handle);
            };
            std::thread{thread_executor}.detach();
        };

        server = rclcpp_action::create_server<action::ClickIk>(
                this,
                "click_ik",
                handle_goal,
                handle_cancel,
                handle_accepted);

        auto iShandle_goal = [this](rclcpp_action::GoalUUID const& uuid, action::IkImageSample_Goal::ConstSharedPtr const& goal) -> rclcpp_action::GoalResponse {
            RCLCPP_INFO(this->get_logger(), "IK Image Sample request received.");
            (void) uuid;
            (void) goal;
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        };


        auto iShandle_cancel = [this](std::shared_ptr<rclcpp_action::ServerGoalHandle<action::IkImageSample>> const& goal_handle) -> rclcpp_action::CancelResponse {
            RCLCPP_INFO(this->get_logger(), "Cancelling IK Image Sample Action.");
            (void) goal_handle;
            return rclcpp_action::CancelResponse::ACCEPT;
        };

        auto iShandle_accepted = [this](std::shared_ptr<rclcpp_action::ServerGoalHandle<action::IkImageSample>> const& goal_handle) -> void {
            if (iSCurrentGoalHandle) {
                auto result = std::make_shared<action::IkImageSample::Result>();
                iSCurrentGoalHandle->abort(result);
            }
            iSCurrentGoalHandle = goal_handle;
            auto thread_executor = [this, goal_handle]() {
                return this->executeIkImageSample(goal_handle);
            };
            std::thread{thread_executor}.detach();
        };

        iSServer = rclcpp_action::create_server<action::IkImageSample>(
                this,
                "ik_image_sample",
                iShandle_goal,
                iShandle_cancel,
                iShandle_accepted);

        mIkSampleClient = create_client<srv::IkSample>("ik_sample");

        mPcSub = create_subscription<sensor_msgs::msg::PointCloud2>("/zed/left/points", 1, [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr const& msg) {
            pointCloudCallback(msg);
        });

        // IK Publisher
        mIkPub = create_publisher<msg::IK>("/ee_pos_cmd", 1);

        // ArmStatus subscriber
        mStatusSub = create_subscription<msg::ArmStatus>("/arm_cmd_status", 1, [this](msg::ArmStatus const& msg) {
            statusCallback(msg);
        });

        mCurrentGoalHandle = nullptr;
    }

    void
    ClickIkNode::executeClickIk(std::shared_ptr<rclcpp_action::ServerGoalHandle<action::ClickIk>> const& goal_handle) {
        RCLCPP_INFO(get_logger(), "Executing goal");
        if (!goal_handle) {
            RCLCPP_WARN(get_logger(), "Invalid ClickIK goal handle");
            return;
        }

        auto const goal = goal_handle->get_goal();
        auto feedback = std::make_shared<action::ClickIk::Feedback>();
        auto result = std::make_shared<action::ClickIk::Result>();
        auto target_point = spiralSearchInImg(static_cast<size_t>(goal->point_in_image_x), static_cast<size_t>(goal->point_in_image_y));

        if (!target_point.has_value()) {
            RCLCPP_WARN(get_logger(), "Target point does not exist.");
            result->success = false;
            goal_handle->canceled(result);
            return; // fix
        }

        geometry_msgs::msg::Pose pose;

        double offset = 0.1; // make sure we don't collide by moving back a little from the target
        pose.position.set__x(target_point.value().x - offset);
        pose.position.set__y(target_point.value().y);
        pose.position.set__z(target_point.value().z);
        SE3d target_pose = SE3Conversions::fromPose(pose);
        target_point->x -= ArmController::END_EFFECTOR_LENGTH;
        SE3d target_transfer = SE3Conversions::fromTfTree(*mTfBuffer, "zed_left_camera_frame", "arm_base_link");
        target_transfer *= target_pose;


        // message.pos = pose;
        // message.target.header.frame_id = "zed_left_camera_frame";
        timer = this->create_wall_timer(std::chrono::milliseconds(10), [this, target_pose, target_transfer, goal_handle, result, feedback]() {
            if (goal_handle->is_canceling()) {
                auto result = std::make_shared<action::ClickIk::Result>();
                result->success = false;
                timer->cancel();
                goal_handle->canceled(result);
                if (mCurrentGoalHandle)
                    mCurrentGoalHandle = nullptr;
                RCLCPP_INFO(get_logger(), "ClickIk Goal Cancelled Successfully.");
                return;
            }
            if (!goal_handle->is_executing()) return;

            float const tolerance = 0.02;
            try {
                SE3d arm_position = SE3Conversions::fromTfTree(*mTfBuffer, "arm_e_link", "zed_left_camera_frame");
                double distance = pow(pow(arm_position.x() + ArmController::END_EFFECTOR_LENGTH - target_pose.x(), 2) + pow(arm_position.y() - target_pose.y(), 2) + pow(arm_position.z() - target_pose.z(), 2), 0.5);
                RCLCPP_INFO(this->get_logger(), "Arm Position: %f, %f, %f", arm_position.x(), arm_position.y(), arm_position.z());
                RCLCPP_INFO(this->get_logger(), "Arm Command: %f, %f, %f", target_transfer.x(), target_transfer.y(), target_transfer.z());
                RCLCPP_INFO(this->get_logger(), "Desired Position: %f, %f, %f", target_pose.x(), target_pose.y(), target_pose.z());

                feedback->distance = static_cast<float>(distance);
                goal_handle->publish_feedback(feedback);

                if (distance < tolerance) {
                    timer->cancel();
                    result->success = true;
                    goal_handle->succeed(result);
                    return;
                }
                msg::IK ik;
                ik.pos.x = target_transfer.x();
                ik.pos.y = target_transfer.y();
                ik.pos.z = target_transfer.z();
                mIkPub->publish(ik);
            } catch (tf2::ExtrapolationException& e) {
                RCLCPP_WARN(this->get_logger(), "ExtrapolationException (due to lag?): %s", e.what());
            }
        });
    }

    void ClickIkNode::abortClickIk() {
        timer->cancel();
        if (mCurrentGoalHandle) {
            auto result = std::make_shared<action::ClickIk::Result>();
            result->success = false;
            mCurrentGoalHandle->abort(result);
            mCurrentGoalHandle = nullptr;
        }
        RCLCPP_INFO(get_logger(), "ClickIk Goal Aborted Successfully.");
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
        std::size_t temp = mNumPoints;

        // Find the smaller of the two box dimensions so we know the max spiral radius
        std::size_t smallDim = std::min(mPointCloudWidth / 2, mPointCloudHeight / 2);

        while (isPointInvalid) {
            // This is the parametric equation to spiral around the center pnt
            currX = static_cast<size_t>(static_cast<double>(xCenter) + std::cos(t * 1.0 / numPts * 2 * M_PI) * static_cast<double>(radius));
            currY = static_cast<size_t>(static_cast<double>(yCenter) + std::sin(t * 1.0 / numPts * 2 * M_PI) * static_cast<double>(radius));

            if (currX > mPointCloudWidth)
                currX = mPointCloudWidth;
            if (currY > mPointCloudHeight)
                currY = mPointCloudWidth;
            if (currX + currY * mPointCloudWidth >= temp)
                return std::nullopt;

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
        (void) temp;
        return std::make_optional<Point>(point);
    }

    auto ClickIkNode::statusCallback(msg::ArmStatus const& msg) -> void {
        if (!msg.status && mCurrentGoalHandle) {
            RCLCPP_WARN(get_logger(), "Arm position unreachable");
            abortClickIk();
        }
    }

    void ClickIkNode::executeIkImageSample(std::shared_ptr<rclcpp_action::ServerGoalHandle<action::IkImageSample>> const& goal_handle) {
        RCLCPP_INFO(get_logger(), "Executing goal");
        if (!goal_handle) {
            RCLCPP_WARN(get_logger(), "Invalid IKImageSample goal handle");
            return;
        }

        auto const goal = goal_handle->get_goal();
        auto result = std::make_shared<action::IkImageSample::Result>();
        result->success.resize(goal->w * goal->h);
        for (int i = 0; i < goal->w; i++) {
            for (int j = 0; j < goal->h; j++) {
                if (goal_handle->is_canceling()) {
                    goal_handle->canceled(result);
                    if (iSCurrentGoalHandle)
                        iSCurrentGoalHandle = nullptr;
                    RCLCPP_INFO(get_logger(), "ClickIk Goal Cancelled Successfully.");
                    return;
                }
                auto target_point = spiralSearchInImg(static_cast<size_t>(goal->scale * i), static_cast<size_t>(goal->scale * j));
                if (!target_point.has_value()) {
                    result->success[i * goal->w + j] = false;
                    continue;
                }

                geometry_msgs::msg::Pose pose;

                double offset = 0.1; // make sure we don't collide by moving back a little from the target
                pose.position.set__x(target_point.value().x - offset);
                pose.position.set__y(target_point.value().y);
                pose.position.set__z(target_point.value().z);
                SE3d target_pose = SE3Conversions::fromPose(pose);
                target_point->x -= ArmController::END_EFFECTOR_LENGTH;
                SE3d target_transfer = SE3Conversions::fromTfTree(*mTfBuffer, "zed_left_camera_frame", "arm_base_link");
                target_transfer *= target_pose;
                srv::IkSample::Request::SharedPtr sendReq = std::make_shared<srv::IkSample::Request>();
                geometry_msgs::msg::Vector3 vec;
                vec.set__x(target_transfer.x());
                vec.set__y(target_transfer.y());
                vec.set__z(target_transfer.z());
                sendReq->set__pos(vec);
                size_t index = j * goal->w + i;
                auto future = mIkSampleClient->async_send_request(sendReq);

                if (future.wait_for(std::chrono::milliseconds(200)) == std::future_status::ready) {
                    result->success[index] = future.get()->valid;
                    RCLCPP_INFO(this->get_logger(), "Point: %zu resolved successfully.", index);
                } else {
                    result->success[index] = false;
                    RCLCPP_INFO(this->get_logger(), "Point: %zu resolved unsuccessfully.", index);
                }
                // RCLCPP_INFO(this->get_logger(), "Requesting sample point: %zu", index);
                // futures.push(mIkSampleClient->async_send_request(sendReq, [this, &resp, index](rclcpp::Client<srv::IkSample>::SharedFuture response) {
                //     resp->success[index] = response.get()->valid;
                //     RCLCPP_DEBUG(this->get_logger(), "Point resolved to %d!", response.get()->valid ? 1 : 0);
                // }));
                // if (futures.front().wait_for(std::chrono::milliseconds(100)) == std::future_status::ready)
                //     futures.pop();
            }
        }
        iSCurrentGoalHandle = nullptr;
        goal_handle->succeed(result);
    }
    // while (!futures.empty()) {
    //     try {
    //         if (futures.front().wait_for(std::chrono::seconds(0)) != std::future_status::ready)
    //             futures.front().wait_for(std::chrono::milliseconds(20));
    //         futures.front().get()->valid;
    //         futures.pop();
    //     } catch (...) {
    //         futures.pop();
    //     }
    //     RCLCPP_INFO(this->get_logger(), "Resolved future for sample point: %zu", bufSize + 1 - futures.size());
    // }

    // void ClickIkNode::imageSampleCallback(srv::IkImageSample::Request::ConstSharedPtr const& req, srv::IkImageSample::Response::SharedPtr const& resp) {
    //     std::queue<rclcpp::Client<srv::IkSample>::SharedFutureAndRequestId> futures;
    //     size_t bufSize = req->w * req->h;
    //     resp->success.resize(req->w * req->h);
    //     for (int i = 0; i < req->w; i++) {
    //         for (int j = 0; j < req->h; j++) {
    //             auto target_point = spiralSearchInImg(static_cast<size_t>(req->scale * i), static_cast<size_t>(req->scale * j));
    //             if (!target_point.has_value()) {
    //                 resp->success[i * req->w + j] = false;
    //                 continue;
    //             }

    //             geometry_msgs::msg::Pose pose;

    //             double offset = 0.1; // make sure we don't collide by moving back a little from the target
    //             pose.position.set__x(target_point.value().x - offset);
    //             pose.position.set__y(target_point.value().y);
    //             pose.position.set__z(target_point.value().z);
    //             SE3d target_pose = SE3Conversions::fromPose(pose);
    //             target_point->x -= ArmController::END_EFFECTOR_LENGTH;
    //             SE3d target_transfer = SE3Conversions::fromTfTree(*mTfBuffer, "zed_left_camera_frame", "arm_base_link");
    //             target_transfer *= target_pose;
    //             srv::IkSample::Request::SharedPtr sendReq = std::make_shared<srv::IkSample::Request>();
    //             geometry_msgs::msg::Vector3 vec;
    //             vec.set__x(target_pose.x());
    //             vec.set__y(target_pose.y());
    //             vec.set__z(target_pose.z());
    //             sendReq->set__pos(vec);
    //             size_t index = j * req->w + i;
    //             auto future = mIkSampleClient->async_send_request(sendReq);

    //             if (future.wait_for(std::chrono::milliseconds(2000)) == std::future_status::ready) {
    //                 resp->success[index] = future.get()->valid;
    //                 RCLCPP_INFO(this->get_logger(), "Point: %zu resolved successfully.", index);
    //             } else {
    //                 resp->success[index] = false;
    //                 RCLCPP_INFO(this->get_logger(), "Point: %zu resolved unsuccessfully.", index);
    //             }
    //             // RCLCPP_INFO(this->get_logger(), "Requesting sample point: %zu", index);
    //             // futures.push(mIkSampleClient->async_send_request(sendReq, [this, &resp, index](rclcpp::Client<srv::IkSample>::SharedFuture response) {
    //             //     resp->success[index] = response.get()->valid;
    //             //     RCLCPP_DEBUG(this->get_logger(), "Point resolved to %d!", response.get()->valid ? 1 : 0);
    //             // }));
    //             // if (futures.front().wait_for(std::chrono::milliseconds(100)) == std::future_status::ready)
    //             //     futures.pop();
    //         }
    //     }
    // while (!futures.empty()) {
    //     try {
    //         if (futures.front().wait_for(std::chrono::seconds(0)) != std::future_status::ready)
    //             futures.front().wait_for(std::chrono::milliseconds(20));
    //         futures.front().get()->valid;
    //         futures.pop();
    //     } catch (...) {
    //         futures.pop();
    //     }
    //     RCLCPP_INFO(this->get_logger(), "Resolved future for sample point: %zu", bufSize + 1 - futures.size());
    // }


} // namespace mrover

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(mrover::ClickIkNode);