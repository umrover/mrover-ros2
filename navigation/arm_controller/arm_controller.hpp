#pragma once
#include "pch.hpp"
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <deque>

namespace mrover {

    class ArmController final : public rclcpp::Node {
        struct ArmPos {
            double x{0}, y{0}, z{0}, pitch{0}, roll{0}, gripper{0};
            [[nodiscard]] auto toSE3() const -> SE3d {
                return SE3d{{x, y, z,}, SO3d{Eigen::Quaterniond{Eigen::AngleAxisd{pitch, R3d::UnitY()} * Eigen::AngleAxisd{roll, R3d::UnitX()}}}};
            }

            auto operator+(R3d offset) const -> ArmPos {
                return {x + offset.x(), y + offset.y(), z + offset.z(), pitch, roll};
            }

            auto operator=(msg::IK ik_target) -> ArmPos& {
                x = ik_target.pos.x;
                y = ik_target.pos.y;
                z = ik_target.pos.z;
                pitch = ik_target.pitch;
                roll = ik_target.roll;
                return *this;
            }
        };

        //double mCarrotTime = 0.033;
        //double mCarrotk = 1;
        rclcpp::Time mPrevTime;
        bool carrot_initialized = false;
        ArmPos mCarrotPos;
        ArmPos mCheckCarrotPos;
        //bool hold = false;
        bool not_initialized = true;
        double mArmTotalError[5];
        double mPrevArmError[5];

        struct JointWrapper {
            struct JointLimits {
                double minPos, maxPos, minVel, maxVel;
                [[nodiscard]] auto posInBounds(double pos) const -> bool {return minPos <= pos && pos <= maxPos;}
                [[nodiscard]] auto velInBounds(double vel) const -> bool {return minPos <= vel && vel <= maxPos;}
            };
            
            JointLimits limits;
            double pos;
        };

        struct ArmPidController {
            double Kp;
            double Ki;
            double Kd;
            double prev_error = 0;
            double error_integral = 0;
            double lim;
            bool first = true;

            ArmPidController(double p, double i, double d, double l)
                : Kp(p), Ki(i), Kd(d), lim(l) { }

            double update(double carrot_error, double dt) {
                error_integral += carrot_error * dt;
                error_integral = std::clamp(error_integral, -1 * lim, lim);
                double derivative = 0;
                if (!first) {
                    derivative = (carrot_error - prev_error)/dt;
                }
                first = false;
                prev_error = carrot_error;
                return (Kp * carrot_error) + (Ki * error_integral) + (Kd * derivative);
            }

            void reset() {
                error_integral = 0;
                prev_error = 0;
                first = true;
            }

        };

        ArmPidController mPIDx{70, 1, 30, 0.2};
        ArmPidController mPIDy{50, 0.00, 30, 0.05};
        ArmPidController mPIDz{300, 0.5, 60, 0.2};
        ArmPidController mPIDpitch{10, 0.0, 3.0, 0.05};
        ArmPidController mPIDroll{10, 0.0, 3.0, 0.05};
        
        // these positional limits are slightly conservative versions of the limits listed in the 2025-26 cdr
        std::unordered_map<std::string, JointWrapper> joints = {
                {"joint_a", {.limits = {.minPos = 0, .maxPos = 0.37, .minVel = -0.05, .maxVel = 0.05}, .pos = 0}},
                {"joint_b", {.limits = {.minPos = -1.1, .maxPos = 0.25, .minVel = -0.05, .maxVel = 0.05}, .pos = 0}},
                {"joint_c", {.limits = {.minPos = -1.0, .maxPos = 3.0, .minVel = -0.03142, .maxVel = 0.03142}, .pos = 0}},
                {"joint_de_pitch", {.limits = {.minPos = -1.75, .maxPos = 1.1, .minVel = -0.2, .maxVel = 0.2}, // pretty conservative limits atm
                                    .pos = 0}},
                {"joint_de_roll", {.limits = {.minPos = -3.14, .maxPos = 3.13, .minVel = -1.0, .maxVel = 1.0}, .pos = 0}},
                {"gripper", {.limits = {.minPos = 0, .maxPos = 0.1, .minVel = -1, .maxVel = 1}, .pos = 0}},
        };

        [[maybe_unused]] rclcpp::Subscription<msg::IK>::SharedPtr mIkSub;
        [[maybe_unused]] rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr mVelSub;
        [[maybe_unused]] rclcpp::Subscription<msg::ControllerState>::SharedPtr mJointSub;

        rclcpp::Publisher<msg::Position>::SharedPtr mPosPub;
        rclcpp::Publisher<msg::Velocity>::SharedPtr mVelPub;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr mEEPathPub;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr mEEPointPub;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr mCtPointPub;
        std::deque<geometry_msgs::msg::PoseStamped> mPathPoses;
        tf2_ros::TransformBroadcaster mTfBroadcaster{this};
        tf2_ros::Buffer mTfBuffer{get_clock()};
        tf2_ros::TransformListener mTfListener{mTfBuffer};
        rclcpp::TimerBase::SharedPtr mTimer;
        rclcpp::Service<srv::IkMode>::SharedPtr mModeServ;

        auto ikPosCalc(ArmPos target) -> std::optional<msg::Position>;
        auto ikVelCalc(geometry_msgs::msg::Twist) -> std::optional<msg::Velocity>;
        auto timerCallback() -> void;
        auto velZeroCheck() -> bool;
        auto carrotPosAdjust(ArmPos &mPos, geometry_msgs::msg::Twist &mVel, double dt, double k) -> void;
        auto carrotPosCheck(ArmPos &mCarrotPos, ArmPos &mCheckCarrotPos, ArmPos &mArmPosCheck) -> void;
        auto visualize_carrot_ee() -> void;
        auto configure_posestamped(geometry_msgs::msg::PoseStamped &p_stamped) -> void;
        auto configure_vis_marker(visualization_msgs::msg::Marker &point,
                                             ArmController::ArmPos &mTargetPos,
                                             float x, float y, float z,
                                             float a, float r, float g, float b) -> void;
        

        ArmPos mArmPos, mTypingOrigin, mPosTarget;
        geometry_msgs::msg::Twist mVelTarget;
        rclcpp::Time mLastUpdate;

        enum class ArmMode : char {
            VELOCITY_CONTROL,
            POSITION_CONTROL,
            TYPING
        };
        ArmMode mArmMode = ArmMode::POSITION_CONTROL;
        static const rclcpp::Duration TIMEOUT;

    public:
        // TODO(quintin): Neven, please load these from config YAML files instead of hard coding. Ideally they would even be computed at runtime. This way you can change the xacro without worry.

        // From: rover.urdf.xacro
        // A is the prismatic joint, B is the first revolute joint, C is the second revolute joint
        static constexpr double LINK_BC = 0.53271;
        static constexpr double LINK_CD = 0.39403;
        static constexpr double END_EFFECTOR_LENGTH = 0.20482814; // from CAD
        static constexpr double JOINT_C_OFFSET = 0.208954;
        static constexpr double JOINT_VEL_THRESH = 0.05;
        static constexpr double MAX_SPEED = 0.1; // in m/s, this is just an estimate

        ArmController();

        void posCallback(msg::IK::ConstSharedPtr const& ik_target);
        void velCallback(geometry_msgs::msg::Twist::ConstSharedPtr const& ik_vel);
        void fkCallback(msg::ControllerState::ConstSharedPtr const& joint_state);
        auto modeCallback(srv::IkMode::Request::ConstSharedPtr const& req, srv::IkMode::Response::SharedPtr const& resp) -> void;
    };

} // namespace mrover