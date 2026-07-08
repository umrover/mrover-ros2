#pragma once

#include <chrono>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <mujoco/mujoco.h>

#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <mrover/msg/controller_state.hpp>
#include <mrover/msg/image_targets.hpp>
#include <mrover/msg/position.hpp>
#include <mrover/msg/throttle.hpp>
#include <mrover/msg/velocity.hpp>
#include <mrover/srv/toggle_object_detector.hpp>

#include "lie.hpp"
#include "viewer.hpp"

namespace mrover::sim {

    using Clock = std::chrono::steady_clock;

    // One controllable joint. The MuJoCo actuator is configured (post-compile) as a
    // native position servo (arm) or velocity servo (drive) so the gain/bias terms are
    // integrated implicitly and stay stable; the node only writes the setpoint to ctrl.
    //
    // Arm joints are position-controlled: position commands set the target directly,
    // while velocity/throttle commands drift the target and the joint holds its last
    // position once commands go stale.
    // Drive joints are velocity-controlled and persist their last command.
    struct Joint {
        enum class Kind { Position,
                          Velocity } kind = Kind::Velocity;

        int actuator = -1; // index into mjData::ctrl / mjData::actuator_force
        int qposAdr = -1;  // index into mjData::qpos
        int dofAdr = -1;   // index into mjData::qvel
        int jointId = -1;  // index into mjModel joint arrays

        double target = 0.0;    // ctrl setpoint (position for arm, velocity for drive)
        double targetVel = 0.0; // arm only: drift rate from velocity/throttle commands
        bool limited = false;
        double lower = 0.0;
        double upper = 0.0;

        Clock::time_point lastCommand = Clock::now();
    };

    // A controller bank (arm or drive): subscribes to throttle/velocity/position and
    // republishes a ControllerState for its joints, matching the embedded firmware.
    struct MotorGroup {
        std::vector<std::string> names;
        rclcpp::Subscription<msg::Throttle>::SharedPtr throttleSub;
        rclcpp::Subscription<msg::Velocity>::SharedPtr velocitySub;
        rclcpp::Subscription<msg::Position>::SharedPtr positionSub;
        rclcpp::Publisher<msg::ControllerState>::SharedPtr controllerStatePub;
    };

    class Simulator final : public rclcpp::Node {
    public:
        Simulator();
        ~Simulator() override;

        Simulator(Simulator const&) = delete;
        auto operator=(Simulator const&) -> Simulator& = delete;
        Simulator(Simulator&&) = delete;
        auto operator=(Simulator&&) -> Simulator& = delete;

        // Advances physics to track wall time and publishes ground truth, TF,
        // controller states, and image targets. Called from the main spin loop.
        auto step() -> void;

        [[nodiscard]] auto targetRate() const -> double { return mTargetUpdateRate; }

    private:
        auto throttlesCallback(msg::Throttle::ConstSharedPtr const& msg) -> void;
        auto velocitiesCallback(msg::Velocity::ConstSharedPtr const& msg) -> void;
        auto positionsCallback(msg::Position::ConstSharedPtr const& msg) -> void;

        template<typename Names, typename Values, typename Fn>
        auto forEachJoint(Names const& names, Values const& values, Fn&& fn) -> void;

        auto applyControls() -> void;
        auto applyViewerTeleop() -> void;
        auto publishGroundTruth() -> void;
        auto publishTransforms() -> void;
        auto publishControllerStates() -> void;
        auto publishImageTargets() -> void;

        [[nodiscard]] auto bodyInWorld(int bodyId) const -> SE3d;

        mjModel* mModel = nullptr;
        mjData* mData = nullptr;

        // Map joint command name (e.g. "joint_a", "front_left") -> live joint.
        std::unordered_map<std::string, Joint> mJoints;
        std::vector<MotorGroup> mMotorGroups;

        // Map object name (e.g. "rover", "bottle") -> its root body id.
        std::unordered_map<std::string, int> mObjectRoots;
        int mRoverBaseBody = -1;

        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr mGroundTruthPub;
        rclcpp::Publisher<msg::ImageTargets>::SharedPtr mImageTargetsPub;
        rclcpp::Service<srv::ToggleObjectDetector>::SharedPtr mStereoToggleServer;
        rclcpp::Service<srv::ToggleObjectDetector>::SharedPtr mImageToggleServer;
        tf2_ros::TransformBroadcaster mTfBroadcaster{this};

        std::unique_ptr<Viewer> mViewer;

        // Settings
        std::string mMapFile = "default_map.yaml";
        double mTargetUpdateRate = 100.0;
        int64_t mMotorTimeoutMs = 100;
        bool mTeleopActive = false;
        double mPublishMalletDistanceThreshold = 3.0;
        double mPublishBottleDistanceThreshold = 3.0;

        Clock::time_point mLastStep = Clock::now();
    };

} // namespace mrover::sim
