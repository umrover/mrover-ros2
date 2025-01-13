#include "arm_controller.hpp"

namespace mrover {

    ArmController::ArmController() : Node{"arm_controller"} {
        mPosPub = create_publisher<msg::Position>("arm_position_cmd", 10);

        mIkSub = create_subscription<msg::IK>("arm_ik", 1, [this](msg::IK::ConstSharedPtr const& msg) {
            ikCallback(msg);
        });

        // Subscription for velocity commands
        mVector3 = create_subscription<geometry_msgs::msg::Vector3>("ee_vel_cmd", 1, [this](geometry_msgs::msg::Vector3::ConstSharedPtr const& msg) {
            velCallback(msg);
        });

        // Subscription for joint states
        mJointState = create_subscription<sensor_msgs::msg::JointState>("arm_joint_data", 1, [this](sensor_msgs::msg::JointState::ConstSharedPtr const& msg) {
            fkCallback(msg);
        });
    }
    auto yawSo3(double r) -> SO3d {
        auto q = Eigen::Quaterniond{Eigen::AngleAxisd{r, R3d::UnitY()}};
        return {q.normalized()};
    }
    auto ArmController::ikCalc(SE3d target) -> std::optional<msg::Position> {
        double x = target.translation().x() - END_EFFECTOR_LENGTH; // shift back by the length of the end effector
        double y = target.translation().y();
        double z = target.translation().z();

        double gamma = 0;
        double x3 = x - LINK_DE * std::cos(gamma);
        double z3 = z - LINK_DE * std::sin(gamma);

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
            q3 >= JOINT_DE_PITCH_MIN && q3 <= JOINT_DE_PITCH_MAX) {
            msg::Position positions;
            positions.names = {"joint_a", "joint_b", "joint_c", "joint_de_pitch"};
            positions.positions = {
                    static_cast<float>(y),
                    static_cast<float>(q1),
                    static_cast<float>(q2),
                    static_cast<float>(q3),
            };
            return positions;
        }
        return std::nullopt;
    }

    void ArmController::velCallback(geometry_msgs::msg::Vector3::ConstSharedPtr const& ik_vel) {
        
        // "Carrot on a stick" approach for controlling arm movement using velocity
        const double increment = 0.1;
        SE3d mCurrentEndEffector;
        SE3d currentPosition = mCurrentEndEffector;

        Eigen::Vector3d velocityVector{ik_vel->x, ik_vel->y, ik_vel->z};

        Eigen::Vector3d incrementVector = velocityVector * increment;
        SE3d newTargetPosition = currentPosition;
        newTargetPosition.translation() = newTargetPosition.translation() + incrementVector;

        std::optional<msg::Position> positions = ikCalc(newTargetPosition);

        if (positions) {
            mPosPub->publish(positions.value());
            mCurrentEndEffector = newTargetPosition;

            // Print the current end effector position after velocity update
            double x_total = newTargetPosition.translation().x();
            double y_total = newTargetPosition.translation().y();
            double z_total = newTargetPosition.translation().z();
            RCLCPP_INFO(get_logger(), "End effector position after velocity update: x=%.3f, y=%.3f, z=%.3f", x_total, y_total, z_total);
        } else {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Failed to calculate IK");
        }
    }


    void ArmController::fkCallback(sensor_msgs::msg::JointState::ConstSharedPtr const& joint_state) {
        SE3d mCurrentEndEffector;

        if (joint_state->position.size() < 4) {
            RCLCPP_ERROR(get_logger(), "Not enough joints");
            return;
        }

        double joint_a = joint_state->position[0]; 
        double joint_b = joint_state->position[1]; 
        double joint_c = joint_state->position[2]; 
        double joint_de_pitch = joint_state->position[3]; 

        joint_c += JOINT_C_OFFSET;

        double x_link_bc = LINK_BC * std::cos(joint_b);
        double z_link_bc = LINK_BC * std::sin(joint_b);

        double x_link_cd = LINK_CD * std::cos(joint_b + joint_c);
        double z_link_cd = LINK_CD * std::sin(joint_b + joint_c);

        double x_link_de = LINK_DE * std::cos(joint_b + joint_c + joint_de_pitch);
        double z_link_de = LINK_DE * std::sin(joint_b + joint_c + joint_de_pitch);

        double x_total = x_link_bc + x_link_cd + x_link_de;
        double z_total = z_link_bc + z_link_cd + z_link_de;
        double y_total = joint_a; 

        mCurrentEndEffector = SE3d{{x_total, y_total, z_total}, SO3d::Identity()};

        RCLCPP_INFO(get_logger(), "End effector position: x=%.3f, y=%.3f, z=%.3f", x_total, y_total, z_total);

    }

    void ArmController::ikCallback(msg::IK::ConstSharedPtr const& ik_target) {
        SE3d targetFrameToArmBaseLink;
        try {
            targetFrameToArmBaseLink = SE3Conversions::fromTfTree(mTfBuffer, ik_target->target.header.frame_id, "arm_base_link");
        } catch (tf2::TransformException const& exception) {
            RCLCPP_WARN_STREAM_THROTTLE(get_logger(), *get_clock(), 1000, std::format("Failed to get transform from {} to arm_base_link: {}", ik_target->target.header.frame_id, exception.what()));
            return;
        }
        SE3d endEffectorInTarget{{ik_target->target.pose.position.x, ik_target->target.pose.position.y, ik_target->target.pose.position.z}, SO3d::Identity()};
        SE3d endEffectorInArmBaseLink = targetFrameToArmBaseLink * endEffectorInTarget;
        SE3Conversions::pushToTfTree(mTfBroadcaster, "arm_target", "arm_base_link", endEffectorInArmBaseLink, get_clock()->now());

        std::optional<msg::Position> positions = ikCalc(endEffectorInArmBaseLink);

        if (positions) {
            mPosPub->publish(positions.value());
        } else {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "IK Position Control Failed");
        }
    }

} // namespace mrover

auto main(int argc, char** argv) -> int {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<mrover::ArmController>());
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}