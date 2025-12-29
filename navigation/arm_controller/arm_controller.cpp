#include "arm_controller.hpp"
#include <rclcpp/logging.hpp>

namespace mrover {
    const rclcpp::Duration ArmController::TIMEOUT = rclcpp::Duration(0, 0.3 * 1e9); // 0.3 seconds

    ArmController::ArmController() : Node{"arm_controller"}, mLastUpdate{get_clock()->now() - TIMEOUT} {
        mPosPub = create_publisher<msg::Position>("arm_position_cmd", 10);
        mVelPub = create_publisher<msg::Velocity>("arm_velocity_cmd", 10);

        mIkSub = create_subscription<msg::IK>("ee_pos_cmd", 1, [this](msg::IK::ConstSharedPtr const& msg) {
            posCallback(msg);
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
    }

    auto ArmController::ikPosCalc(ArmPos target) -> std::optional<msg::Position> {
        double x = target.x; // shift back by the length of the end effector
        double y = target.y;
        double z = target.z;

        double gamma = -target.pitch;
        double x3 = x - END_EFFECTOR_LENGTH * std::cos(gamma);
        double z3 = z - END_EFFECTOR_LENGTH * std::sin(gamma);

        double C = std::sqrt(x3 * x3 + z3 * z3);
        double alpha = std::acos((LINK_BC * LINK_BC + LINK_CD * LINK_CD - C * C) / (2 * LINK_BC * LINK_CD));
        double beta = std::acos((LINK_BC * LINK_BC + C * C - LINK_CD * LINK_CD) / (2 * LINK_BC * C));
        double thetaA = std::atan(z3 / x3) + beta;
        double thetaB = -1 * (std::numbers::pi - alpha);
        double thetaC = gamma - thetaA - thetaB;

        double q1 = -thetaA;
        double q2 = -thetaB + JOINT_C_OFFSET;
        double q3 = -thetaC - JOINT_C_OFFSET;

        // if any angles are infinite, it means math failed (usually due to the position being out of reach of the arm)
        if (!std::isfinite(q1) || !std::isfinite(q2) || !std::isfinite(q3)) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Couldn't solve position IK");
            return std::nullopt;
        }

        msg::Position positions;
        positions.name = {"joint_a", "joint_b", "joint_c", "joint_de_pitch", "joint_de_roll"};
        positions.position = {
            static_cast<float>(y),
            static_cast<float>(q1),
            static_cast<float>(q2),
            static_cast<float>(q3),
            static_cast<float>(target.roll),
        };

        for (size_t i = 0; i < positions.name.size(); ++i) {
            auto it = joints.find(positions.name[i]);
            // hopefully this will never happen, but just in case
            if (it == joints.end()) {
                RCLCPP_WARN_STREAM_THROTTLE(get_logger(), *get_clock(), 1000, "Unknown joint \"" << positions.name[i] << "\"");
                return std::nullopt;
            }

            if (!it->second.limits.posInBounds(positions.position[i])) {
                RCLCPP_WARN_STREAM_THROTTLE(get_logger(), *get_clock(), 1000, "Position for joint " << positions.name[i] << " not within limits!");
                return std::nullopt;
            }
        }
        
        return positions;
    }

    auto ArmController::ikVelCalc(geometry_msgs::msg::Twist vel) -> std::optional<msg::Velocity> {
        double x_3 = mArmPos.x;
        double z_3 = mArmPos.z;

        double x_2 = x_3 - END_EFFECTOR_LENGTH * std::cos(mArmPos.pitch);
        double x_2_vel = vel.linear.x + END_EFFECTOR_LENGTH * std::sin(mArmPos.pitch) * vel.angular.y; // derivative of x_2
        double z_2 = z_3 + END_EFFECTOR_LENGTH * std::sin(mArmPos.pitch);
        double z_2_vel = vel.linear.z + END_EFFECTOR_LENGTH * std::cos(mArmPos.pitch) * vel.angular.y; // derivative of z_2
        double c2 = x_2 * x_2 + z_2 * z_2;
        double c_vel = (2 * x_2 * x_2_vel + 2 * z_2 * z_2_vel) / (2 * std::sqrt(c2)); // derivative of c
        double a2 = LINK_BC * LINK_BC;
        double b2 = LINK_CD * LINK_CD;

        double num = c2 + a2 - b2;
        double denom = 2 * LINK_BC * std::sqrt(c2);
        double joint_b_vel = 1 / std::sqrt(1 - (num / denom) * (num / denom));
        num = c2 * c_vel - c_vel * (a2 - b2);
        denom *= std::sqrt(c2);
        joint_b_vel *= (num / denom);
        num = z_2_vel * x_2 - z_2 * x_2_vel;
        denom = x_2 * x_2;
        joint_b_vel -= (1 / (1 + (z_2 * z_2) / (x_2 * x_2))) * (num / denom);

        num = c2 - a2 - b2;
        denom = 2 * LINK_BC * LINK_CD;
        double joint_c_vel = -1 / std::sqrt((1 - (num * num) / (denom * denom)));
        num = 2 * std::sqrt(c2) * c_vel;
        joint_c_vel *= (num / denom);

        double joint_de_pitch_vel = vel.angular.y - joint_b_vel - joint_c_vel;

        if (!std::isfinite(joint_b_vel) || !std::isfinite(joint_c_vel) || !std::isfinite(joint_de_pitch_vel)) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Couldn't solve velocity IK");
            return std::nullopt;
        }

        msg::Velocity velocities;
        velocities.name = {"joint_a", "joint_b", "joint_c", "joint_de_pitch", "joint_de_roll"};
        velocities.velocity = {
            static_cast<float>(vel.linear.y),
            static_cast<float>(joint_b_vel),
            static_cast<float>(joint_c_vel),
            static_cast<float>(joint_de_pitch_vel),
            static_cast<float>(vel.angular.x),
        };

        double scaleFactor = 1;
        for (size_t i = 0; i < velocities.name.size(); ++i) {
            auto it = joints.find(velocities.name[i]);
            if (it == joints.end()) {
                RCLCPP_WARN_STREAM_THROTTLE(get_logger(), *get_clock(), 1000, "Unknown Joint\"" << velocities.name[i] << "\"");
                return std::nullopt;
            }

            if ((velocities.velocity[i] > 0 && it->second.limits.maxPos - it->second.pos < JOINT_VEL_THRESH) ||
                (velocities.velocity[i] < 0 && it->second.pos - it->second.limits.minPos < JOINT_VEL_THRESH)) {
                RCLCPP_WARN_STREAM_THROTTLE(get_logger(), *get_clock(), 1000, "Joint " << velocities.name[i] << " too close to limit for velocity command");
                return std::nullopt;
            }
            scaleFactor = std::max(scaleFactor, velocities.velocity[i] / (velocities.velocity[i] > 0 ? it->second.limits.maxVel : it->second.limits.minVel));
        }

        // scale down all velocities so that we don't exceed motor velocity limits
        if (scaleFactor > 1)
            RCLCPP_INFO_STREAM_THROTTLE(get_logger(), *get_clock(), 500, "Commanded velocity too high. Scaling down by factor of " << scaleFactor);
        for (auto& v : velocities.velocity)
            v = static_cast<float>(v / scaleFactor);

        return velocities;
    }

    void ArmController::velCallback(geometry_msgs::msg::Twist::ConstSharedPtr const& ik_vel) {
        mVelTarget = *ik_vel;
        mVelTarget.linear.x *= MAX_SPEED;
        mVelTarget.linear.y *= MAX_SPEED;
        mVelTarget.linear.z *= MAX_SPEED;
        if (mArmMode == ArmMode::VELOCITY_CONTROL)
            mLastUpdate = get_clock()->now();
        else
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 100, "Received velocity command in position mode!");
    }

    void ArmController::fkCallback(sensor_msgs::msg::JointState::ConstSharedPtr const& joint_state) {
        // update joint positions stored in ArmController class
        for (size_t i = 0; i < joint_state->name.size(); ++i) {
            auto it = joints.find(joint_state->name[i]);
            if (it == joints.end()) {
                RCLCPP_WARN_STREAM_THROTTLE(get_logger(), *get_clock(), 1000, "Unknown joint \"" << joint_state->name[i] << "\"");
                continue;
            }
            it->second.pos = joint_state->position[i];
        }

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
        x += END_EFFECTOR_LENGTH * std::cos(angle);
        z += END_EFFECTOR_LENGTH * std::sin(angle);
        mArmPos = {x, y, z, -angle, joint_state->position[4], std::max(joint_state->position[5], 0.0)};

	SE3Conversions::pushToTfTree(mTfBroadcaster, "arm_fk", "arm_base_link", mArmPos.toSE3(), get_clock()->now());
    }

    void ArmController::posCallback(msg::IK::ConstSharedPtr const& ik_target) {
        mPosTarget = *ik_target;
        SE3Conversions::pushToTfTree(mTfBroadcaster, "arm_target", "arm_base_link", mPosTarget.toSE3(), get_clock()->now());
        if (mArmMode == ArmMode::POSITION_CONTROL || mArmMode == ArmMode::TYPING)
                mLastUpdate = get_clock()->now();
        else
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 100, "Received position command in velocity mode!");
    }

    auto ArmController::timerCallback() -> void {
        msg::Position mCurrPos;
        mCurrPos.name = {"joint_a", "joint_b", "joint_c", "joint_de_pitch", "joint_de_roll"};
        mCurrPos.position = {
            static_cast<float>(joints["joint_a"].pos),
            static_cast<float>(joints["joint_b"].pos),
            static_cast<float>(joints["joint_c"].pos),
            static_cast<float>(joints["joint_de_pitch"].pos),
            static_cast<float>(joints["joint_de_roll"].pos),
        };

        if (get_clock()->now() - mLastUpdate > TIMEOUT) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 100, "IK Timed Out");
            if(!mPosFallback) mPosFallback = mCurrPos;
            mPosPub->publish(mPosFallback.value());
            return;
        }

        if (mArmMode == ArmMode::POSITION_CONTROL) {
            auto positions = ikPosCalc(mPosTarget);
            SE3Conversions::pushToTfTree(mTfBroadcaster, "arm_target", "arm_base_link", mPosTarget.toSE3(), get_clock()->now());
            if (positions) {
                mPosPub->publish(positions.value());
                mPosFallback = std::nullopt;
            } else {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Position IK failed!");
                if(!mPosFallback) mPosFallback = mCurrPos;
                mPosPub->publish(mPosFallback.value());
            }
        } else if (mArmMode == ArmMode::VELOCITY_CONTROL) {
            // TODO: Determine joint velocities that cancels out arm sag
            auto velocities = ikVelCalc(mVelTarget);
            if (velocities &&
                !(
                    velocities->velocity[0] == 0 &&
                    velocities->velocity[1] == 0 &&
                    velocities->velocity[2] == 0 &&
                    velocities->velocity[3] == 0 &&
                    velocities->velocity[4] == 0
                )
            ) {
                mVelPub->publish(velocities.value());
                mPosFallback = std::nullopt;
            } else {
                if(!velocities) RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Velocity IK failed!");
                if(!mPosFallback) mPosFallback = mCurrPos;
                mPosPub->publish(mPosFallback.value());
            }
        } else { // typing mode
            msg::Position positions;
            positions.name = {"joint_a", "gripper"};
            positions.position = {
                static_cast<float>(mPosTarget.y + mTypingOrigin.y),
                static_cast<float>(mPosTarget.z + mTypingOrigin.gripper),
            };

            // bounds checking and such
            for (size_t i = 0; i < positions.name.size(); ++i) {
                auto it = joints.find(positions.name[i]);
                // hopefully this will never happen, but just in case
                if (it == joints.end()) {
                    RCLCPP_WARN_STREAM_THROTTLE(get_logger(), *get_clock(), 1000, "Unknown joint \"" << positions.name[i] << "\"");
                    return;
                }

                if (!it->second.limits.posInBounds(positions.position[i])) {
                    RCLCPP_WARN_STREAM_THROTTLE(get_logger(), *get_clock(), 1000, "Position for joint " << positions.name[i] << " not within limits! (" << positions.position[i] << ")");
                    RCLCPP_WARN_STREAM_THROTTLE(get_logger(), *get_clock(), 1000, "Typing IK failed");
                    return;
                }
            }
            mPosFallback = std::nullopt;
            mPosPub->publish(positions);
        }
    }

    auto ArmController::modeCallback(srv::IkMode::Request::ConstSharedPtr const& req, srv::IkMode::Response::SharedPtr const& resp) -> void {
        if (req->mode == srv::IkMode::Request::POSITION_CONTROL) {
            mArmMode = ArmMode::POSITION_CONTROL;
            RCLCPP_INFO(get_logger(), "IK Switching to Position Control Mode");
        } else if (req->mode == srv::IkMode::Request::VELOCITY_CONTROL) {
            mArmMode = ArmMode::VELOCITY_CONTROL;
            RCLCPP_INFO(get_logger(), "IK Switching to Velocity Control Mode");
        } else { // typing mode
            mArmMode = ArmMode::TYPING;
            RCLCPP_INFO(get_logger(), "IK Switching to Typing (position) Mode");
            // set reference point for typing
            mTypingOrigin = mArmPos;
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
