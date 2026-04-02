#include "arm_controller.hpp"
#include "mrover/msg/detail/velocity__struct.hpp"
#include <algorithm>
#include <geometry_msgs/msg/detail/twist__struct.hpp>
#include <rclcpp/logging.hpp>

namespace mrover {
    const rclcpp::Duration ArmController::TIMEOUT = rclcpp::Duration(0, 0.3 * 1e9);

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

        rclcpp::on_shutdown([this]() {
            msg::Velocity stop;
            stop.names = {"joint_a","joint_b","joint_c","joint_de_pitch","joint_de_roll"};
            stop.velocities = {0, 0, 0, 0, 0};
            mVelPub->publish(stop);
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
        double val1 = ((LINK_BC * LINK_BC + LINK_CD * LINK_CD - C * C) / (2 * LINK_BC * LINK_CD));
        double val2 = ((LINK_BC * LINK_BC + C * C - LINK_CD * LINK_CD) / (2 * LINK_BC * C));

        val1 = std::clamp(val1,-1.0,1.0);
        val2 = std::clamp(val2,-1.0,1.0);

        double alpha = std::acos(val1);
        double beta = std::acos(val2);

        double thetaA_1 = std::atan2(z3, x3) + beta;
        double thetaB_1 = -(std::numbers::pi - alpha);   // elbow up

        double thetaA_2 = std::atan2(z3, x3) - beta;
        double thetaB_2 = (std::numbers::pi - alpha);    // elbow down — THIS IS THE FIX

        // compute both full solutions
        double thetaC_1 = gamma - thetaA_1 - thetaB_1;
        double thetaC_2 = gamma - thetaA_2 - thetaB_2;

        double q1_1 = -thetaA_1, q2_1 = -thetaB_1 + JOINT_C_OFFSET, q3_1 = -thetaC_1 - JOINT_C_OFFSET;
        double q1_2 = -thetaA_2, q2_2 = -thetaB_2 + JOINT_C_OFFSET, q3_2 = -thetaC_2 - JOINT_C_OFFSET;

        // pick whichever solution has joint_b in bounds [-0.9, 0]
        double q1, q2, q3;

        if (q1_1 >= -0.9 && q1_1 <= 0) {
    q1 = q1_1;
    q2 = q2_1; 
    q3 = q3_1;
} else {
    q1 = q1_2;
    q2 = q2_2; 
    q3 = q3_2;
}

        RCLCPP_INFO(get_logger(), "IK computed: joint_a=%.4f joint_b=%.4f joint_c=%.4f joint_de_pitch=%.4f joint_de_roll=%.4f",
        (float)y, (float)q1, (float)q2, (float)q3, (float)target.roll);

        // if any angles are infinite, it means math failed (usually due to the position being out of reach of the arm)
        if (!std::isfinite(q1) || !std::isfinite(q2) || !std::isfinite(q3)) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Couldn't solve position IK");
            return std::nullopt; 
        }

        msg::Position positions;
        positions.names = {"joint_a", "joint_b", "joint_c", "joint_de_pitch", "joint_de_roll"};
        positions.positions = {
            static_cast<float>(y),
            static_cast<float>(q1),
            static_cast<float>(q2),
            static_cast<float>(q3),
            static_cast<float>(target.roll),
        };

        for (size_t i = 0; i < positions.names.size(); ++i) {
            auto it = joints.find(positions.names[i]);
            // hopefully this will never happen, but just in case
            if (it == joints.end()) {
                RCLCPP_WARN_STREAM_THROTTLE(get_logger(), *get_clock(), 1000, "Unknown joint \"" << positions.names[i] << "\"");
                return std::nullopt;
            }

            if (!it->second.limits.posInBounds(positions.positions[i])) {
                RCLCPP_WARN(get_logger(),
                    "Joint %s out of bounds: %.4f not in [%.4f, %.4f]",
                    positions.names[i].c_str(),
                    positions.positions[i],
                    it->second.limits.minPos,
                    it->second.limits.maxPos
                );
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
        velocities.names = {"joint_a", "joint_b", "joint_c", "joint_de_pitch", "joint_de_roll"};
        velocities.velocities = {
            static_cast<float>(vel.linear.y),
            static_cast<float>(joint_b_vel),
            static_cast<float>(joint_c_vel),
            static_cast<float>(joint_de_pitch_vel),
            static_cast<float>(vel.angular.x),
        };

        double scaleFactor = 1;
        for (size_t i = 0; i < velocities.names.size(); ++i) {
            auto it = joints.find(velocities.names[i]);
            if (it == joints.end()) {
                RCLCPP_WARN_STREAM_THROTTLE(get_logger(), *get_clock(), 1000, "Unknown Joint\"" << velocities.names[i] << "\"");
                return std::nullopt;
            }

            if ((velocities.velocities[i] > 0 && it->second.limits.maxPos - it->second.pos < JOINT_VEL_THRESH) ||
                (velocities.velocities[i] < 0 &&
                it->second.pos - it->second.limits.minPos < JOINT_VEL_THRESH)) {

                scaleFactor = std::max(scaleFactor, 3.0);
            }


            scaleFactor = std::max(scaleFactor, velocities.velocities[i] / (velocities.velocities[i] > 0 ? it->second.limits.maxVel : it->second.limits.minVel));
        }

        // scale down all velocities so that we don't exceed motor velocity limits
        if (scaleFactor > 1)
            RCLCPP_INFO_STREAM_THROTTLE(get_logger(), *get_clock(), 500, "Commanded velocity too high. Scaling down by factor of " << scaleFactor);
        for (auto& v : velocities.velocities)
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
    ArmPos temp;
    temp = *ik_target;

    double gamma = -temp.pitch;
    double x3 = temp.x - END_EFFECTOR_LENGTH * std::cos(gamma);
    double z3 = temp.z - END_EFFECTOR_LENGTH * std::sin(gamma);
    double C = std::sqrt(x3 * x3 + z3 * z3);

    double maxReach = LINK_BC + LINK_CD;
    double minReach = std::abs(LINK_BC - LINK_CD);

    if (C > maxReach || C < minReach) {
        RCLCPP_WARN(get_logger(), "Unreachable target: C=%.4f not in [%.4f, %.4f]", C, minReach, maxReach);
        return;
    }

    constexpr double Y_MIN = -0.1;
    constexpr double Y_MAX = 0.45;
    if (temp.y < Y_MIN || temp.y > Y_MAX) {
        RCLCPP_WARN(get_logger(), "Unreachable target: y=%.4f out of bounds [%.4f, %.4f]",
            temp.y, Y_MIN, Y_MAX);
        return;
    }

    mPosTarget = *ik_target;
    SE3Conversions::pushToTfTree(mTfBroadcaster, "arm_target", "arm_base_link", mPosTarget.toSE3(), get_clock()->now());
    mLastUpdate = get_clock()->now();
}

    auto ArmController::timerCallback() -> void {
        if (get_clock()->now() - mLastUpdate > TIMEOUT) {
            msg::Velocity stop_message;
            stop_message.names = {"joint_a","joint_b","joint_c","joint_de_pitch","joint_de_roll"};
            stop_message.velocities ={0,0,0,0,0};
            mVelPub->publish((stop_message));
            return;

        }

        if (mArmMode == ArmMode::POSITION_CONTROL) {
            auto positions = ikPosCalc(mPosTarget);
            SE3Conversions::pushToTfTree(mTfBroadcaster, "arm_target", "arm_base_link", mPosTarget.toSE3(), get_clock()->now());
            if (positions) {
                mPosPub->publish(positions.value());
            } else {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Position IK failed!");
            }

        } else if (mArmMode == ArmMode::VELOCITY_CONTROL) {
            
            double dx = mPosTarget.x - mArmPos.x;
            double dy = mPosTarget.y - mArmPos.y;
            double dz = mPosTarget.z - mArmPos.z;

            double d_pitch = mPosTarget.pitch - mArmPos.pitch;
            double d_roll = mPosTarget.roll - mArmPos.roll;

            double distance = (dx*dx) + (dy*dy) + (dz*dz) + (d_pitch*d_pitch) + (d_roll*d_roll);

            if (sqrt(distance) < 0.01){
                msg::Velocity vel_zero;
                vel_zero.names = {"joint_a","joint_b","joint_c","joint_de_pitch","joint_de_roll"};
                vel_zero.velocities = {0,0,0,0,0};
                mVelPub->publish(vel_zero);
                return;
            }
            
            const double Kp = 2;
            geometry_msgs::msg::Twist command;

            auto clamp = [](double v, double max) {
                return std::clamp(v, -max, max);
            };

            const double MAX_CART_VEL = 3; // meters/sec
            const double MAX_ANG_VEL  = 0.5;  // rad/sec

            command.linear.x  = clamp(Kp * dx, MAX_CART_VEL);
            command.linear.y  = clamp(Kp * dy, MAX_CART_VEL);
            command.linear.z  = clamp(Kp * dz, MAX_CART_VEL);

            command.angular.y = clamp(Kp * d_pitch, MAX_ANG_VEL);
            command.angular.x = clamp(Kp * d_roll,  MAX_ANG_VEL);
            command.angular.z = 0;
            

            auto velocities = ikVelCalc(command);

            if (!velocities){
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 500,
                             "Velocity IK failed during tracking");
                return;
            }

            mVelPub->publish(velocities.value());

        } else { // typing mode
            msg::Position positions;
            positions.names = {"joint_a", "gripper"};
            positions.positions = {        
                static_cast<float>(mPosTarget.y + mTypingOrigin.y),
                static_cast<float>(mPosTarget.z + mTypingOrigin.gripper),
            };

            // bounds checking and such
            for (size_t i = 0; i < positions.names.size(); ++i) {
                auto it = joints.find(positions.names[i]);
                // hopefully this will never happen, but just in case
                if (it == joints.end()) {
                    RCLCPP_WARN_STREAM_THROTTLE(get_logger(), *get_clock(), 1000, "Unknown joint \"" << positions.names[i] << "\"");
                    return;
                }

                if (!it->second.limits.posInBounds(positions.positions[i])) {
                    RCLCPP_WARN_STREAM_THROTTLE(get_logger(), *get_clock(), 1000, "Position for joint " << positions.names[i] << " not within limits! (" << positions.positions[i] << ")");
                    RCLCPP_WARN_STREAM_THROTTLE(get_logger(), *get_clock(), 1000, "Typing IK failed");
                    return;
                }
            }

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
