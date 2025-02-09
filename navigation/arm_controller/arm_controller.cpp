#include "arm_controller.hpp"

namespace mrover {
    const rclcpp::Duration ArmController::TIMEOUT = rclcpp::Duration(1, 0); // one second

    ArmController::ArmController() : Node{"arm_controller"} {
        mPosPub = create_publisher<msg::Position>("arm_position_cmd", 10);

        mIkSub = create_subscription<msg::IK>("ee_pos_cmd", 1, [this](msg::IK::ConstSharedPtr const& msg) {
            ikCallback(msg);
        });

        mVelSub = create_subscription<geometry_msgs::msg::Twist>("ee_vel_cmd", 1, [this](geometry_msgs::msg::Twist::ConstSharedPtr const& msg) {
            velCallback(msg);
        });

        mJointSub = create_subscription<sensor_msgs::msg::JointState>("arm_joint_data", 1, [this](sensor_msgs::msg::JointState::ConstSharedPtr const& msg) {
            fkCallback(msg);
        });

        mTimer = create_wall_timer(std::chrono::milliseconds(33), [this]() {
            timerCallback();
        });

        mModeServ = create_service<srv::IkMode>("ik_mode", [this](srv::IkMode::Request::ConstSharedPtr const& req, srv::IkMode::Response::SharedPtr const& resp) {
            modeCallback(req, resp);
        });
    
        mLastUpdate = get_clock()->now(); // TODO: maybe subtract timeout here to make sure arm is initially timed out
    }

    auto yawSo3(double r) -> SO3d {
        auto q = Eigen::Quaterniond{Eigen::AngleAxisd{r, R3d::UnitY()}};
        return {q.normalized()};
    }

    auto ArmController::ikCalc(ArmPos target) -> std::optional<msg::Position> {
        double x = target.x; // shift back by the length of the end effector
        double y = target.y;
        double z = target.z;

        double gamma = -target.pitch;
        double x3 = x - (LINK_DE + END_EFFECTOR_LENGTH) * std::cos(gamma);
        double z3 = z - (LINK_DE + END_EFFECTOR_LENGTH) * std::sin(gamma);

        double C = std::sqrt(x3 * x3 + z3 * z3);
        double alpha = std::acos((LINK_BC * LINK_BC + LINK_CD * LINK_CD - C * C) / (2 * LINK_BC * LINK_CD));
        double beta = std::acos((LINK_BC * LINK_BC + C * C - LINK_CD * LINK_CD) / (2 * LINK_BC * C));
        double thetaA = std::atan(z3 / x3) + beta;
        double thetaB = -1 * (std::numbers::pi - alpha);
        double thetaC = gamma - thetaA - thetaB;

        double q1 = -thetaA;
        double q2 = -thetaB + JOINT_C_OFFSET;
        double q3 = -thetaC - JOINT_C_OFFSET;

        if (std::isfinite(q1) && std::isfinite(q2) && std::isfinite(q3) &&
            y >= JOINT_A_MIN && y <= JOINT_A_MAX &&
            q1 >= JOINT_B_MIN && q1 <= JOINT_B_MAX &&
            q2 >= JOINT_C_MIN && q2 <= JOINT_C_MAX &&
            q3 >= JOINT_DE_PITCH_MIN && q3 <= JOINT_DE_PITCH_MAX &&
            target.roll >= JOINT_DE_ROLL_MIN && target.roll <= JOINT_DE_ROLL_MAX) {
            msg::Position positions;
            positions.names = {"joint_a", "joint_b", "joint_c", "joint_de_pitch", "joint_de_roll"};
            positions.positions = {
                    static_cast<float>(y),
                    static_cast<float>(q1),
                    static_cast<float>(q2),
                    static_cast<float>(q3),
                    static_cast<float>(target.roll),
            };
            return positions;
        }
        return std::nullopt;
    }

    void ArmController::velCallback(geometry_msgs::msg::Twist::ConstSharedPtr const& ik_vel) {
        mVelTarget = {ik_vel->linear.x, ik_vel->linear.y, ik_vel->linear.z};
        mPitchVel = ik_vel->angular.y;
        mRollVel = ik_vel->angular.x;
        if (mArmMode == ArmMode::VELOCITY_CONTROL)
            mLastUpdate = get_clock()->now();
        else
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 100, "Received velocity command in posiition mode!");
        // R3d new_vel{ik_vel->x, ik_vel->y, ik_vel->z};
        // if (!mVelTarget.isZero() && new_vel.isZero()) { // stop arm
        //     mPosTarget = mArmPos;
        // }
        // mVelTarget = new_vel;
        // mLastUpdate = get_clock()->now();
    }

    void ArmController::fkCallback(sensor_msgs::msg::JointState::ConstSharedPtr const& joint_state) {
        double y = joint_state->position[0]; // joint a position
        // joint b position
        double angle = -joint_state->position[1];
        double x = LINK_BC * std::cos(angle);
        double z = LINK_BC * std::sin(angle);
        // joint c position
        angle -= joint_state->position[2] - JOINT_C_OFFSET;
        x += LINK_CD * std::cos(angle);
        z += LINK_CD * std::sin(angle);
        // joint de position
        angle -= joint_state->position[3] + JOINT_C_OFFSET;
        x += (LINK_DE + END_EFFECTOR_LENGTH) * std::cos(angle);
        z += (LINK_DE + END_EFFECTOR_LENGTH) * std::sin(angle);
        mArmPos = {x, y, z, -angle, joint_state->position[4]};
	SE3Conversions::pushToTfTree(mTfBroadcaster, "arm_fk", "arm_base_link", mArmPos.toSE3(), get_clock()->now());
    }

    void ArmController::ikCallback(msg::IK::ConstSharedPtr const& ik_target) {
        mPosTarget = *ik_target;
        SE3Conversions::pushToTfTree(mTfBroadcaster, "arm_target", "arm_base_link", mPosTarget.toSE3(), get_clock()->now());
        if (mArmMode == ArmMode::POSITION_CONTROL)
                mLastUpdate = get_clock()->now();
        else
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 100, "Received position command in velocity mode!");
        // SE3d targetFrameToArmBaseLink;
        // try {
        //     targetFrameToArmBaseLink = SE3Conversions::fromTfTree(mTfBuffer, ik_target->target.header.frame_id, "arm_base_link");
        // } catch (tf2::TransformException const& exception) {
        //     RCLCPP_WARN_STREAM_THROTTLE(get_logger(), *get_clock(), 1000, std::format("Failed to get transform from {} to arm_base_link: {}", ik_target->target.header.frame_id, exception.what()));
        //     return;
        // }
        // SE3d endEffectorInTarget{{ik_target->target.pose.position.x, ik_target->target.pose.position.y, ik_target->target.pose.position.z}, 
        //                          SO3d{ik_target->target.pose.orientation.x, ik_target->target.pose.orientation.y, ik_target->target.pose.orientation.z, ik_target->target.pose.orientation.w}};
        // SE3d endEffectorInArmBaseLink = targetFrameToArmBaseLink * endEffectorInTarget;
        // SE3Conversions::pushToTfTree(mTfBroadcaster, "arm_target", "arm_base_link", endEffectorInArmBaseLink, get_clock()->now());
        // mPosTarget = endEffectorInArmBaseLink;
        // mLastUpdate = get_clock()->now();
    }

    auto ArmController::timerCallback() -> void {
        if (get_clock()->now() - mLastUpdate > TIMEOUT) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 100, "IK Timed Out");
            return;
        }

        ArmPos target;
        if (mArmMode == ArmMode::POSITION_CONTROL) {
            target = mPosTarget;
        } else {
            target = mPosTarget + mVelTarget * 0.01;
            target.pitch += mPitchVel * 0.05;
            target.roll += mRollVel * 0.05;
        }
        SE3Conversions::pushToTfTree(mTfBroadcaster, "arm_target", "arm_base_link", target.toSE3(), get_clock()->now());
        auto positions = ikCalc(target);
        if (positions) {
            mPosPub->publish(positions.value());
            // if successful, move the target to the new spot (relevant for velocity control)
            mPosTarget = target;
        } else {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "IK Failed");
            // if IK failed in velocity mode, we go back to where the target was pre-increment (this should always be a reachable
            // position because mPosTarget only gets updated in velocity mode if the position is reachable)
            // this should hopefully get us to the (approximately) closest point that is reachable
            if (mArmMode == ArmMode::VELOCITY_CONTROL) {
                positions = ikCalc(target);
                if (!positions) // surely this will never happen right
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Undo velocity control failed");
                else
                    mPosPub->publish(positions.value());
            }
        }
    }

    auto ArmController::modeCallback(srv::IkMode::Request::ConstSharedPtr const& req, srv::IkMode::Response::SharedPtr const& resp) -> void {
        if (req->mode == req->POSITION_CONTROL) {
            mArmMode = ArmMode::POSITION_CONTROL;
            RCLCPP_INFO(get_logger(), "IK Switching to Position Control Mode");
        } else {
            mArmMode = ArmMode::VELOCITY_CONTROL;
            RCLCPP_INFO(get_logger(), "IK Switching to Velocity Control Mode");
            // when we switch to velocity control mode, set the target to the current position
            mPosTarget = mArmPos;
        }
        resp->success = true;
    }
} // namespace mrover

auto main(int argc, char** argv) -> int {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<mrover::ArmController>());
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
