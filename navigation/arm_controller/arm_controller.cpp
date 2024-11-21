#include "arm_controller.hpp"

namespace mrover {

    ArmController::ArmController() : Node{"arm_controller"} {
        mPosPub = create_publisher<msg::Position>("arm_position_cmd", 10);

        mIkSub = create_subscription<msg::IK>("arm_ik", 1, [this](msg::IK::ConstSharedPtr const& msg) {
            ikCallback(msg);
        });

        // TODO: create a new subscription for the topic "ee_vel_cmd"

        // TODO: create a new subscription for the topic "arm_joint_data" - this will be used to determine the current position of the arm
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
            positions.names = {"joint_a", "joint_b", "joint_c", "joint_de_pitch", "joint_de_roll"};
            positions.positions = {
                    static_cast<float>(y),
                    static_cast<float>(q1),
                    static_cast<float>(q2),
                    static_cast<float>(q3),
                    0.f
            };
            return positions;
        }
        return std::nullopt;
    }

    void ArmController::velCallback(geometry_msgs::msg::Vector3::ConstSharedPtr const& ik_vel) {
        // TODO: implement this function using the "carrot on a stick" approach
    }

    void ArmController::fkCallback(sensor_msgs::msg::JointState::ConstSharedPtr const& joint_state) {
        // TODO: implement forward kinematics here to determine the position of the end of the arm based on the joint angles
        // HINT: compute the position of each link of the arm one at a time
        // NOTE: you will need to add the constant JOINT_C_OFFSET to the angle for joint c
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
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "IK Positon Control Failed");
        }
    }

} // namespace mrover

auto main(int argc, char** argv) -> int {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<mrover::ArmController>());
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
