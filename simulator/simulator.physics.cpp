#include "simulator.hpp"

namespace mrover {

    // Determines the timestep for the physics update.
    // Need a relatively high frequency to achive a stable simulation.
    constexpr unsigned int PHYSICS_UPDATE_HZ = 120;
    // Set to a very high value, so that if the computer can not keep up the above in realtime, it slows down instead of dropping frames.
    // In a video game we do not want this behavior but this is robotics so we have to be deterministic.
    // See: https://stackoverflow.com/questions/12778229/what-does-step-mean-in-stepsimulation-and-what-do-its-parameters-mean-in-bulle
    // Important formula that needs to hold true to avoid dropping: timeStep < maxSubSteps * fixedTimeStep
    constexpr int MAX_SUB_STEPS = 1024;

    constexpr double TAU = 2 * std::numbers::pi;

    auto btTransformToSe3(btTransform const& transform) -> SE3d {
        btVector3 const& p = transform.getOrigin();
        btQuaternion const& q = transform.getRotation();
        // Note: Must convert the Bullet quaternion (floats) to a normalized Eigen quaternion (doubles).
        //       Otherwise the normality check will fail in the SO3 constructor.
        return SE3d{R3d{p.x(), p.y(), p.z()}, Eigen::Quaterniond{q.w(), q.x(), q.y(), q.z()}.normalized()};
    }

    auto Simulator::initPhysics() -> void {
        RCLCPP_INFO_STREAM(get_logger(), std::format("Using Bullet Physics Version: {}", btGetVersion()));

        mCollisionConfig = std::make_unique<btDefaultCollisionConfiguration>();

        mDispatcher = std::make_unique<btCollisionDispatcher>(mCollisionConfig.get());

        mOverlappingPairCache = std::make_unique<btHashedOverlappingPairCache>();

        mBroadphase = std::make_unique<btDbvtBroadphase>(mOverlappingPairCache.get());

        mSolver = std::make_unique<btMultiBodyMLCPConstraintSolver>(new btDantzigSolver{});
        // mSolver = std::make_unique<btMultiBodyConstraintSolver>();

        mDynamicsWorld = std::make_unique<btMultiBodyDynamicsWorld>(mDispatcher.get(), mBroadphase.get(), mSolver.get(), mCollisionConfig.get());
        // mDynamicsWorld->getSolverInfo().m_minimumSolverBatchSize = 1;
    }

    auto Simulator::physicsUpdate(Clock::duration dt) -> void {
        mDynamicsWorld->setGravity(mGravityAcceleration);

        // Make the rocker and bogie try to always return to their initial positions
        // They can still move, so they act as a suspension system
        if (auto it = mUrdfs.find("rover"); it != mUrdfs.end()) {
            URDF const& rover = it->second;

            for (auto const& name: {"left_rocker_link", "right_rocker_link"}) {
                int linkIndex = rover.linkNameToMeta.at(name).index;
                auto* motor = std::bit_cast<btMultiBodyJointMotor*>(rover.physics->getLink(linkIndex).m_userPtr);
                motor->setMaxAppliedImpulse(0.5);
                motor->setPositionTarget(0);
            }
        }

        float updateDuration = std::clamp(std::chrono::duration_cast<std::chrono::duration<float>>(dt).count(), 0.0f, 0.1f);
        int simStepCount = mDynamicsWorld->stepSimulation(updateDuration, MAX_SUB_STEPS, 1.f / PHYSICS_UPDATE_HZ);

        if (auto* mlcpSolver = dynamic_cast<btMLCPSolver*>(mDynamicsWorld->getConstraintSolver())) {
            if (int fallbackCount = mlcpSolver->getNumFallbacks()) {
                RCLCPP_WARN_STREAM_THROTTLE(get_logger(), *get_clock(), 1000, std::format("MLCP solver failed {} times", fallbackCount));
            }
            mlcpSolver->setNumFallbacks(0);
        }

        if (!mHeadless && simStepCount && simStepCount > MAX_SUB_STEPS) {
            int droppedSteps = simStepCount - MAX_SUB_STEPS;
            RCLCPP_ERROR_STREAM(get_logger(), std::format("Dropped {} simulation steps out of {}! This should never happen unless you have a literal potato of a computer OR someone wrote bad code in the iteration loop", droppedSteps, simStepCount));
        }

        linksToTfUpdate();

        gpsAndImusUpdate(dt);

        motorStatusUpdate();
    }

    auto Simulator::linksToTfUpdate() -> void {

        for (auto const& [name, urdf]: mUrdfs) {
            // NOLINTNEXTLINE(misc-no-recursion)
            auto publishLink = [&](auto&& self, urdf::LinkConstSharedPtr const& link) -> void {
                if (link->parent_joint) {
                    int index = urdf.linkNameToMeta.at(link->name).index;
                    // TODO(quintin): figure out why we need to negate rvector
                    btTransform parentToChild{urdf.physics->getParentToLocalRot(index), -urdf.physics->getRVector(index)};
                    SE3d childInParent = btTransformToSe3(parentToChild.inverse());

                    SE3Conversions::pushToTfTree(mTfBroadcaster, link->name, link->getParent()->name, childInParent, get_clock()->now());
                }

                SE3d modelInMap = btTransformToSe3(urdf.physics->getBaseWorldTransform());
                SE3Conversions::pushToTfTree(mTfBroadcaster, std::format("{}_truth", name), "map", modelInMap, get_clock()->now());

                if (name == "rover") SE3Conversions::pushToTfTree(mTfBroadcaster, "base_link", "map", modelInMap, get_clock()->now());

                for (urdf::JointSharedPtr const& child_joint: link->child_joints) {
                    self(self, urdf.model.getLink(child_joint->child_link_name));
                }
            };
            publishLink(publishLink, urdf.model.getRoot());
        }

        if (auto roverOpt = getUrdf("rover")) {
            URDF const& rover = *roverOpt;

            msg::ImageTargets targets;

            auto publishModel = [&](std::string const& modelName, double threshold) {
                if (auto modelOpt = getUrdf(modelName)) {
                    URDF const& model = *modelOpt;

                    SE3d modelInMap = btTransformToSe3(model.physics->getBaseWorldTransform());
                    SE3d roverInMap = btTransformToSe3(rover.physics->getBaseWorldTransform());

                    R3d roverToModel = modelInMap.translation() - roverInMap.translation();
                    double roverDistanceToModel = roverToModel.norm();
                    roverToModel /= roverDistanceToModel;
                    R3d roverForward = roverInMap.rotation().matrix().col(0);
                    double roverDotModel = roverToModel.dot(roverForward);

                    double angleToModel = std::acos(roverDotModel);
                    angleToModel = std::copysign(angleToModel, roverForward.cross(roverToModel).z());

                    if (angleToModel < TAU / 8 && angleToModel > -TAU / 8) {

                        if (roverDistanceToModel < threshold) {
                            SE3Conversions::pushToTfTree(mTfBroadcaster, modelName, "map", modelInMap, get_clock()->now());
                        }

                        if (roverDistanceToModel < threshold * 2) {
                            msg::ImageTarget target;
                            target.name = modelName;
                            target.bearing = static_cast<float>(angleToModel); // TODO: make bearing negative if needed
                            if (angleToModel < TAU / 8 && angleToModel > -TAU / 8) {
                                targets.targets.push_back(target);
                            }
                        }
                    }
                }
            };

            if (mPublishBottleDistanceThreshold > 0) publishModel("bottle", mPublishBottleDistanceThreshold);
            if (mPublishHammerDistanceThreshold > 0) publishModel("hammer", mPublishHammerDistanceThreshold);

            mImageTargetsPub->publish(targets);
        }
    }

    auto URDF::linkInWorld(std::string const& linkName) const -> SE3d {
        int index = linkNameToMeta.at(linkName).index;
        return btTransformToSe3(index == -1 ? physics->getBaseWorldTransform() : physics->getLink(index).m_cachedWorldTransform);
    }

} // namespace mrover
