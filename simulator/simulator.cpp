#include "simulator.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <numbers>
#include <unordered_set>
#include <utility>

#include "parameter.hpp"
#include "scene.hpp"

namespace mrover::sim {

    namespace {
        constexpr double TAU = 2 * std::numbers::pi;
        constexpr char ROVER_PREFIX[] = "rover/";
        constexpr std::size_t ROVER_PREFIX_LEN = sizeof(ROVER_PREFIX) - 1;

        // joint command name -> URDF link the joint moves (and thus body "rover/<link>").
        std::vector<std::pair<std::string, std::string>> const& msgToUrdf() {
            static std::vector<std::pair<std::string, std::string>> const map = {
                    {"joint_a", "arm_a_link"},
                    {"joint_b", "arm_b_link"},
                    {"joint_c", "arm_c_link"},
                    {"joint_de_pitch", "arm_d_link"},
                    {"joint_de_roll", "arm_e_link"},
                    {"gripper", "arm_gripper_link"},
                    {"front_left", "front_left_wheel_link"},
                    {"middle_left", "center_left_wheel_link"},
                    {"back_left", "back_left_wheel_link"},
                    {"front_right", "front_right_wheel_link"},
                    {"middle_right", "center_right_wheel_link"},
                    {"back_right", "back_right_wheel_link"},
            };
            return map;
        }

        std::vector<std::string> const ARM_NAMES = {"joint_a", "joint_b", "joint_c", "joint_de_pitch", "joint_de_roll", "gripper"};
        std::vector<std::string> const DRIVE_NAMES = {"front_right", "middle_right", "back_right", "front_left", "middle_left", "back_left"};

        // Control/stability constants. The converted URDF gives many links tiny bound
        // inertias, so we lean on armature (rotor inertia reflected through the gearbox)
        // and the implicitfast integrator to keep the servos stable.
        constexpr double ARM_KP = 80.0;
        constexpr double ARM_KD = 8.0;
        constexpr double ARM_ARMATURE = 0.1;
        constexpr double DRIVE_KV = 15.0;
        constexpr double DRIVE_ARMATURE = 0.05;
        // The rover's rocker-bogie/lambda suspension is a closed-loop linkage in reality,
        // but URDF/MJCF can only express an open tree, so the suspension links have no
        // constraint closing the loop and flop under gravity. Until we add equality
        // constraints, stiff springs (toward the assembled qpos0 pose) hold the linkage
        // roughly rigid; damping keeps them from oscillating.
        constexpr double SUSPENSION_STIFFNESS = 600.0;
        constexpr double SUSPENSION_DAMPING = 20.0;
        constexpr double PASSIVE_DAMPING = 0.2; // actuated joints' baseline damping
        constexpr double WHEEL_THROTTLE_SPEED = 10.0; // throttle 1.0 -> rad/s
        constexpr double TELEOP_WHEEL_SPEED = 6.0; // keyboard drive, rad/s
    } // namespace

    Simulator::Simulator()
        : Node{"simulator", rclcpp::NodeOptions{}
                                    .allow_undeclared_parameters(true)
                                    .automatically_declare_parameters_from_overrides(true)} {
        int motorTimeoutMs = 100;
        std::vector<ParameterWrapper> params = {
                {"sim_map", mMapFile, std::string{"default_map.yaml"}},
                {"target_update_rate", mTargetUpdateRate, 100.0},
                {"motor_timeout", motorTimeoutMs, 100},
                {"mallet_distance_threshold", mPublishMalletDistanceThreshold, 3.0},
                {"bottle_distance_threshold", mPublishBottleDistanceThreshold, 3.0},
        };
        ParameterWrapper::declareParameters(this, params);
        mMotorTimeoutMs = motorTimeoutMs;

        // Drive every controllable rover joint with a torque motor.
        std::vector<std::string> actuatedLinks;
        actuatedLinks.reserve(msgToUrdf().size());
        for (auto const& [msgName, urdfLink]: msgToUrdf()) actuatedLinks.push_back(urdfLink);

        mModel = buildSceneFromMap(mMapFile, {{"rover", actuatedLinks}});
        mData = mj_makeData(mModel);

        // Implicit-in-velocity integration plus damping on the passive suspension keeps
        // the articulated rover stable; actuated joints get armature + servo gains below.
        mModel->opt.integrator = mjINT_IMPLICITFAST;
        for (int j = 0; j < mModel->njnt; ++j) {
            if (mModel->jnt_type[j] == mjJNT_FREE) continue;
            mModel->dof_damping[mModel->jnt_dofadr[j]] = PASSIVE_DAMPING;
        }

        RCLCPP_INFO_STREAM(get_logger(), std::format("MuJoCo scene '{}': {} bodies, {} dofs, {} actuators, timestep {:.4f}s",
                                                     mMapFile, mModel->nbody, mModel->nv, mModel->nu, mModel->opt.timestep));

        // Resolve each command name to its actuator + joint state, and configure the
        // actuator as a native position (arm) or velocity (drive) servo.
        std::array<std::string, 6> const armSet = {"joint_a", "joint_b", "joint_c", "joint_de_pitch", "joint_de_roll", "gripper"};
        std::unordered_set<int> actuatedJoints;
        for (auto const& [msgName, urdfLink]: msgToUrdf()) {
            std::string bodyName = std::string{ROVER_PREFIX} + urdfLink;
            int bodyId = mj_name2id(mModel, mjOBJ_BODY, bodyName.c_str());
            if (bodyId < 0) {
                RCLCPP_WARN_STREAM(get_logger(), std::format("No body '{}' for joint '{}'", bodyName, msgName));
                continue;
            }
            if (mModel->body_jntnum[bodyId] < 1) {
                RCLCPP_WARN_STREAM(get_logger(), std::format("Body '{}' has no joint for '{}'", bodyName, msgName));
                continue;
            }
            int actId = mj_name2id(mModel, mjOBJ_ACTUATOR, bodyName.c_str());
            if (actId < 0) {
                RCLCPP_WARN_STREAM(get_logger(), std::format("No actuator '{}' for joint '{}'", bodyName, msgName));
                continue;
            }
            int jntId = mModel->body_jntadr[bodyId];
            int dofAdr = mModel->jnt_dofadr[jntId];
            bool isArm = std::find(armSet.begin(), armSet.end(), msgName) != armSet.end();

            Joint joint;
            joint.kind = isArm ? Joint::Kind::Position : Joint::Kind::Velocity;
            joint.actuator = actId;
            joint.jointId = jntId;
            joint.qposAdr = mModel->jnt_qposadr[jntId];
            joint.dofAdr = dofAdr;
            joint.limited = mModel->jnt_limited[jntId] != 0;
            joint.lower = mModel->jnt_range[2 * jntId + 0];
            joint.upper = mModel->jnt_range[2 * jntId + 1];

            mjtNum* gain = mModel->actuator_gainprm + actId * mjNGAIN;
            mjtNum* bias = mModel->actuator_biasprm + actId * mjNBIAS;
            std::fill(gain, gain + mjNGAIN, 0.0);
            std::fill(bias, bias + mjNBIAS, 0.0);
            if (isArm) {
                // force = kp*ctrl - kp*pos - kd*vel  =>  kp*(ctrl - pos) - kd*vel
                mModel->actuator_gaintype[actId] = mjGAIN_FIXED;
                gain[0] = ARM_KP;
                mModel->actuator_biastype[actId] = mjBIAS_AFFINE;
                bias[1] = -ARM_KP;
                bias[2] = -ARM_KD;
                mModel->dof_armature[dofAdr] = ARM_ARMATURE;
            } else {
                // force = kv*ctrl - kv*vel  =>  kv*(ctrl - vel)
                mModel->actuator_gaintype[actId] = mjGAIN_FIXED;
                gain[0] = DRIVE_KV;
                mModel->actuator_biastype[actId] = mjBIAS_AFFINE;
                bias[2] = -DRIVE_KV;
                mModel->dof_armature[dofAdr] = DRIVE_ARMATURE;
            }
            actuatedJoints.insert(jntId);
            mJoints.emplace(msgName, joint);
        }

        // Hold the passive (un-actuated, non-free) suspension joints with stiff springs so
        // the open-tree linkage doesn't sag/collapse under gravity. See SUSPENSION_*.
        for (int j = 0; j < mModel->njnt; ++j) {
            if (mModel->jnt_type[j] == mjJNT_FREE) continue;
            if (actuatedJoints.contains(j)) continue;
            mModel->jnt_stiffness[j] = SUSPENSION_STIFFNESS;
            mModel->dof_damping[mModel->jnt_dofadr[j]] = SUSPENSION_DAMPING;
        }

        mj_forward(mModel, mData);

        // Arm servos hold their spawn pose from the first step; drive servos start idle.
        for (auto& [name, joint]: mJoints) {
            joint.target = joint.kind == Joint::Kind::Position ? mData->qpos[joint.qposAdr] : 0.0;
            mData->ctrl[joint.actuator] = joint.target;
        }

        // Cache each top-level object's root body (parent == world) for truth frames.
        for (int b = 1; b < mModel->nbody; ++b) {
            if (mModel->body_parentid[b] != 0) continue;
            char const* name = mj_id2name(mModel, mjOBJ_BODY, b);
            if (!name) continue;
            std::string_view view{name};
            std::size_t slash = view.find('/');
            if (slash == std::string_view::npos) continue;
            std::string object{view.substr(0, slash)};
            mObjectRoots.emplace(object, b);
        }
        mRoverBaseBody = mj_name2id(mModel, mjOBJ_BODY, "rover/base_link");

        mGroundTruthPub = create_publisher<nav_msgs::msg::Odometry>("ground_truth", 1);
        mImageTargetsPub = create_publisher<msg::ImageTargets>("objects", 1);

        auto addGroup = [this](std::string_view groupName, std::string_view stateTopic, std::vector<std::string> names,
                               std::string_view thr, std::string_view vel, std::string_view pos) {
            MotorGroup& group = mMotorGroups.emplace_back();
            group.names = std::move(names);
            group.controllerStatePub = create_publisher<msg::ControllerState>(std::string{stateTopic}, 1);
            group.throttleSub = create_subscription<msg::Throttle>(std::format("/{}{}", groupName, thr), 1,
                                                                   [this](msg::Throttle::ConstSharedPtr const& msg) { throttlesCallback(msg); });
            group.velocitySub = create_subscription<msg::Velocity>(std::format("/{}{}", groupName, vel), 1,
                                                                   [this](msg::Velocity::ConstSharedPtr const& msg) { velocitiesCallback(msg); });
            group.positionSub = create_subscription<msg::Position>(std::format("/{}{}", groupName, pos), 1,
                                                                   [this](msg::Position::ConstSharedPtr const& msg) { positionsCallback(msg); });
        };
        addGroup("arm", "arm_controller_state", ARM_NAMES, "_thr_cmd", "_vel_cmd", "_pos_cmd");
        addGroup("drive", "drive_controller_state", DRIVE_NAMES, "_throttle_cmd", "_velocity_cmd", "_position_cmd");

        // The detectors live elsewhere in sim; accept toggles so callers don't error.
        mStereoToggleServer = create_service<srv::ToggleObjectDetector>(
                "toggle_stereo_object_detector",
                [](srv::ToggleObjectDetector::Request::ConstSharedPtr const&, srv::ToggleObjectDetector::Response::SharedPtr const& response) {
                    response->success = true;
                });
        mImageToggleServer = create_service<srv::ToggleObjectDetector>(
                "toggle_image_object_detector",
                [](srv::ToggleObjectDetector::Request::ConstSharedPtr const&, srv::ToggleObjectDetector::Response::SharedPtr const& response) {
                    response->success = true;
                });

        mViewer = std::make_unique<Viewer>(mModel);
        RCLCPP_INFO_STREAM(get_logger(), "Viewer open: orbit/pan/zoom with mouse, drive with WASD/arrows, Q/Esc to quit");

        mLastStep = Clock::now();
    }

    Simulator::~Simulator() {
        if (mData) mj_deleteData(mData);
        if (mModel) mj_deleteModel(mModel);
    }

    template<typename Names, typename Values, typename Fn>
    auto Simulator::forEachJoint(Names const& names, Values const& values, Fn&& fn) -> void {
        if (names.size() != values.size()) {
            RCLCPP_WARN_STREAM_THROTTLE(get_logger(), *get_clock(), 1000, "Mismatched joint names and values; ignoring");
            return;
        }
        Clock::time_point now = Clock::now();
        for (std::size_t i = 0; i < names.size(); ++i) {
            auto it = mJoints.find(names[i]);
            if (it == mJoints.end()) {
                RCLCPP_WARN_STREAM_THROTTLE(get_logger(), *get_clock(), 1000, std::format("Unknown joint '{}'", names[i]));
                continue;
            }
            Joint& joint = it->second;
            joint.lastCommand = now;
            fn(joint, static_cast<double>(values[i]));
        }
    }

    auto Simulator::throttlesCallback(msg::Throttle::ConstSharedPtr const& msg) -> void {
        forEachJoint(msg->names, msg->throttles, [](Joint& joint, double throttle) {
            // Throttle drives velocity: arm joints drift their position target, drive
            // joints spin at a fraction of their max wheel speed.
            if (joint.kind == Joint::Kind::Position) joint.targetVel = throttle;
            else joint.target = throttle * WHEEL_THROTTLE_SPEED;
        });
    }

    auto Simulator::velocitiesCallback(msg::Velocity::ConstSharedPtr const& msg) -> void {
        forEachJoint(msg->names, msg->velocities, [](Joint& joint, double velocity) {
            if (joint.kind == Joint::Kind::Position) joint.targetVel = velocity;
            else joint.target = velocity;
        });
    }

    auto Simulator::positionsCallback(msg::Position::ConstSharedPtr const& msg) -> void {
        forEachJoint(msg->names, msg->positions, [](Joint& joint, double position) {
            // Drive joints are continuous; position commands only make sense for the arm.
            if (joint.kind != Joint::Kind::Position) return;
            joint.target = position;
            joint.targetVel = 0.0;
        });
    }

    auto Simulator::applyControls() -> void {
        double timestep = mModel->opt.timestep;
        Clock::time_point now = Clock::now();
        for (auto& [name, joint]: mJoints) {
            if (joint.kind == Joint::Kind::Position) {
                // Stale velocity/throttle commands stop drifting so the joint holds.
                auto ageMs = std::chrono::duration_cast<std::chrono::milliseconds>(now - joint.lastCommand).count();
                if (joint.targetVel != 0.0 && ageMs <= mMotorTimeoutMs) {
                    joint.target += joint.targetVel * timestep;
                    if (joint.limited) joint.target = std::clamp(joint.target, joint.lower, joint.upper);
                }
            }
            mData->ctrl[joint.actuator] = joint.target;
        }
    }

    auto Simulator::bodyInWorld(int bodyId) const -> SE3d {
        R3d p{mData->xpos[3 * bodyId + 0], mData->xpos[3 * bodyId + 1], mData->xpos[3 * bodyId + 2]};
        // MuJoCo stores quaternions as [w, x, y, z].
        S3d q{mData->xquat[4 * bodyId + 0], mData->xquat[4 * bodyId + 1], mData->xquat[4 * bodyId + 2], mData->xquat[4 * bodyId + 3]};
        return SE3d{p, q.normalized()};
    }

    auto Simulator::step() -> void {
        Clock::time_point now = Clock::now();
        double elapsed = std::chrono::duration<double>(now - mLastStep).count();
        mLastStep = now;

        double timestep = mModel->opt.timestep;
        int substeps = std::clamp(static_cast<int>(std::lround(elapsed / timestep)), 1, 64);

        applyViewerTeleop();
        for (int i = 0; i < substeps; ++i) {
            applyControls();
            mj_step(mModel, mData);
        }

        publishGroundTruth();
        publishTransforms();
        publishControllerStates();
        publishImageTargets();

        if (mViewer) {
            mViewer->render(mModel, mData);
            if (!mViewer->isOpen()) rclcpp::shutdown();
        }
    }

    auto Simulator::applyViewerTeleop() -> void {
        if (!mViewer) return;
        double forward = mViewer->driveForward();
        double turn = mViewer->driveTurn();
        bool active = forward != 0.0 || turn != 0.0;

        // Only commandeer the wheels while a drive key is held; on release, stop once
        // and then hand control back to ROS so the viewer can watch autonomy/teleop.
        if (!active) {
            if (mTeleopActive) {
                forward = turn = 0.0;
                mTeleopActive = false;
            } else {
                return;
            }
        } else {
            mTeleopActive = true;
        }

        // Differential drive: same-sign velocity on both sides goes straight (verified
        // against ground truth), so steer by adding the turn term across sides.
        auto setSide = [&](std::vector<std::string> const& wheels, double velocity) {
            for (std::string const& wheel: wheels) {
                if (auto it = mJoints.find(wheel); it != mJoints.end()) it->second.target = velocity;
            }
        };
        static std::vector<std::string> const leftWheels = {"front_left", "middle_left", "back_left"};
        static std::vector<std::string> const rightWheels = {"front_right", "middle_right", "back_right"};
        setSide(leftWheels, (forward - turn) * TELEOP_WHEEL_SPEED);
        setSide(rightWheels, (forward + turn) * TELEOP_WHEEL_SPEED);
    }

    auto Simulator::publishGroundTruth() -> void {
        if (mRoverBaseBody < 0) return;

        SE3d baseInMap = bodyInWorld(mRoverBaseBody);
        std::array<mjtNum, 6> velocity{};
        mj_objectVelocity(mModel, mData, mjOBJ_BODY, mRoverBaseBody, velocity.data(), /*flg_local=*/0);

        nav_msgs::msg::Odometry odometry;
        odometry.header.stamp = get_clock()->now();
        odometry.header.frame_id = "map";
        R3d p = baseInMap.translation();
        odometry.pose.pose.position.x = p.x();
        odometry.pose.pose.position.y = p.y();
        odometry.pose.pose.position.z = p.z();
        S3d q = baseInMap.quat();
        odometry.pose.pose.orientation.w = q.w();
        odometry.pose.pose.orientation.x = q.x();
        odometry.pose.pose.orientation.y = q.y();
        odometry.pose.pose.orientation.z = q.z();
        odometry.twist.twist.angular.x = velocity[0];
        odometry.twist.twist.angular.y = velocity[1];
        odometry.twist.twist.angular.z = velocity[2];
        odometry.twist.twist.linear.x = velocity[3];
        odometry.twist.twist.linear.y = velocity[4];
        odometry.twist.twist.linear.z = velocity[5];
        mGroundTruthPub->publish(odometry);
    }

    auto Simulator::publishTransforms() -> void {
        rclcpp::Time stamp = get_clock()->now();

        for (auto const& [object, bodyId]: mObjectRoots)
            SE3Conversions::pushToTfTree(mTfBroadcaster, object + "_truth", "map", bodyInWorld(bodyId), stamp);

        // Full link tree only for the rover.
        for (int b = 1; b < mModel->nbody; ++b) {
            char const* name = mj_id2name(mModel, mjOBJ_BODY, b);
            if (!name) continue;
            std::string_view view{name};
            if (!view.starts_with(ROVER_PREFIX)) continue;

            std::string childFrame{view.substr(ROVER_PREFIX_LEN)};
            int parent = mModel->body_parentid[b];
            if (parent == 0) {
                SE3Conversions::pushToTfTree(mTfBroadcaster, "base_link", "map", bodyInWorld(b), stamp);
                continue;
            }
            char const* parentName = mj_id2name(mModel, mjOBJ_BODY, parent);
            if (!parentName) continue;
            std::string_view parentView{parentName};
            std::string parentFrame{parentView.starts_with(ROVER_PREFIX) ? parentView.substr(ROVER_PREFIX_LEN) : parentView};

            SE3d childInParent = bodyInWorld(parent).inverse() * bodyInWorld(b);
            SE3Conversions::pushToTfTree(mTfBroadcaster, childFrame, parentFrame, childInParent, stamp);
        }
    }

    auto Simulator::publishControllerStates() -> void {
        for (MotorGroup& group: mMotorGroups) {
            msg::ControllerState state;
            state.header.stamp = get_clock()->now();
            for (std::string const& name: group.names) {
                auto it = mJoints.find(name);
                if (it == mJoints.end()) continue;
                Joint const& joint = it->second;

                state.names.push_back(name);
                state.states.emplace_back("Armed");
                state.errors.emplace_back("None");

                std::uint8_t limits = 0b000;
                if (mModel->jnt_limited[joint.jointId]) {
                    double pos = mData->qpos[joint.qposAdr];
                    double lower = mModel->jnt_range[2 * joint.jointId + 0];
                    double upper = mModel->jnt_range[2 * joint.jointId + 1];
                    constexpr double OFFSET = 0.05;
                    if (pos < lower + OFFSET) limits |= 0b010;
                    if (pos > upper - OFFSET) limits |= 0b001;
                }
                state.limits_hit.push_back(limits);

                state.positions.push_back(static_cast<float>(mData->qpos[joint.qposAdr]));
                state.velocities.push_back(static_cast<float>(mData->qvel[joint.dofAdr]));
                state.currents.push_back(static_cast<float>(mData->actuator_force[joint.actuator]));
            }
            group.controllerStatePub->publish(state);
        }
    }

    auto Simulator::publishImageTargets() -> void {
        if (mRoverBaseBody < 0) return;

        rclcpp::Time stamp = get_clock()->now();
        SE3d roverInMap = bodyInWorld(mRoverBaseBody);
        msg::ImageTargets targets;

        auto considerObject = [&](std::string const& object, double threshold) {
            if (threshold <= 0) return;
            auto it = mObjectRoots.find(object);
            if (it == mObjectRoots.end()) return;

            SE3d modelInMap = bodyInWorld(it->second);
            R3d roverToModel = modelInMap.translation() - roverInMap.translation();
            double distance = roverToModel.norm();
            if (distance < 1e-6) return;
            roverToModel /= distance;

            R3d roverForward = roverInMap.rotation().matrix().col(0);
            double dot = std::clamp(roverToModel.dot(roverForward), -1.0, 1.0);
            double angle = std::acos(dot);
            angle = std::copysign(angle, roverForward.cross(roverToModel).z());

            if (angle >= TAU / 8 || angle <= -TAU / 8) return;

            if (distance < threshold)
                SE3Conversions::pushToTfTree(mTfBroadcaster, object, "map", modelInMap, stamp);

            if (distance < threshold * 2) {
                msg::ImageTarget target;
                target.name = object;
                target.bearing = static_cast<float>(angle);
                targets.targets.push_back(target);
            }
        };

        considerObject("bottle", mPublishBottleDistanceThreshold);
        considerObject("mallet", mPublishMalletDistanceThreshold);

        mImageTargetsPub->publish(targets);
    }

} // namespace mrover::sim
