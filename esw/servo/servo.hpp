#pragma once

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <mrover/msg/servo_configure.hpp>
#include <mrover/msg/servo_in.hpp>
#include <mrover/msg/servo_out.hpp>
#include <parameter.hpp>
#include <units.hpp>

namespace mrover {

    class Servo {
        static constexpr uint8_t ADDR_OPERATING_MODE = 11;
        static constexpr uint8_t ADDR_CURRENT_LIMIT = 38;
        static constexpr uint8_t ADDR_TORQUE_ENABLE = 64;
        static constexpr uint8_t ADDR_POSITION_D_GAIN = 80;
        static constexpr uint8_t ADDR_POSITION_I_GAIN = 82;
        static constexpr uint8_t ADDR_POSITION_P_GAIN = 84;
        static constexpr uint8_t ADDR_PROFILE_ACCELERATION = 108;
        static constexpr uint8_t ADDR_PROFILE_VELOCITY = 112;
        static constexpr uint8_t ADDR_GOAL_POSITION = 116;

        static constexpr int32_t SERVO_TICKS_PER_REV = 4096;
        static constexpr double TAU = 2.0 * M_PI;

        rclcpp::Node::SharedPtr mNode;
        std::string mServoName;
        uint8_t mServoID{0};
        double mGearRatio{1.0};

        Radians mGoalPosition{0.0};
        Radians mBootPosition{0.0};

        std::atomic<bool> mHasReceivedFirstMessage{false};
        std::atomic<double> mRadianOffset{0.0};
        std::atomic<double> mCachedRads{0.0};

        rclcpp::Publisher<msg::ServoConfigure>::SharedPtr mConfigPub;
        rclcpp::Publisher<msg::ServoIn>::SharedPtr mCmdPub;
        rclcpp::Subscription<msg::ServoOut>::SharedPtr mStateSub;

    public:
        Servo(rclcpp::Node::SharedPtr node, std::string servoName)
            : mNode(std::move(node)), mServoName(std::move(servoName)) {
            mConfigPub = mNode->create_publisher<msg::ServoConfigure>("/u2d2/configure", rclcpp::QoS(10).transient_local());
            mCmdPub = mNode->create_publisher<msg::ServoIn>("/u2d2/" + mServoName + "/in", 10);

            mStateSub = mNode->create_subscription<msg::ServoOut>(
                    "/u2d2/" + mServoName + "/out", 10,
                    [this](msg::ServoOut::ConstSharedPtr const& msg) {
                        Radians const rawRads = ticksToRads(static_cast<int32_t>(msg->position));

                        if (!mHasReceivedFirstMessage.exchange(true)) {
                            mRadianOffset.store(mBootPosition.get() - rawRads.get());
                        }

                        mCachedRads.store(rawRads.get());
                    });

            initParametersAndHardware();
        }

        void setGoalPosition(Radians rads) {
            if (rads.get() < 0.0) rads = Radians{0.0};
            mGoalPosition = rads;
            Radians const hardwareRads{rads.get() - mRadianOffset.load()};
            int32_t const hardwareTicks = radsToTicks(hardwareRads);
            publishWrite(ADDR_GOAL_POSITION, 4, static_cast<uint32_t>(hardwareTicks));
        }

        [[nodiscard]] auto getPosition() const -> Radians {
            return Radians{mCachedRads.load() + mRadianOffset.load()};
        }

    private:
        [[nodiscard]] auto radsToTicks(Radians rads) const -> int32_t {
            double const motorRevs = (rads.get() / TAU) * mGearRatio;
            return static_cast<int32_t>(motorRevs * SERVO_TICKS_PER_REV);
        }

        [[nodiscard]] auto ticksToRads(int32_t ticks) const -> Radians {
            double const motorRevs = static_cast<double>(ticks) / SERVO_TICKS_PER_REV;
            double const jointRevs = motorRevs / mGearRatio;
            return Radians{jointRevs * TAU};
        }

        void initParametersAndHardware() {
            int id;
            double position_p, position_i, position_d;
            double current_limit;
            double profile_acceleration, profile_velocity;
            double boot_position;

            std::vector<ParameterWrapper> parameters = {
                    {mServoName + ".id", id, 1},
                    {mServoName + ".gear_ratio", mGearRatio, 1.0},
                    {mServoName + ".position_p", position_p, 400.0},
                    {mServoName + ".position_i", position_i, 0.0},
                    {mServoName + ".position_d", position_d, 0.0},
                    {mServoName + ".current_limit", current_limit, 1000.0},
                    {mServoName + ".profile_acceleration", profile_acceleration, 10.0},
                    {mServoName + ".profile_velocity", profile_velocity, 100.0},
                    {mServoName + ".boot_position", boot_position, 0.0}};
            ParameterWrapper::declareParameters(mNode.get(), parameters);
            mServoID = static_cast<uint8_t>(id);
            mBootPosition = Radians{boot_position};

            // register with u2d2 node
            msg::ServoConfigure configMsg;
            configMsg.name = mServoName;
            configMsg.id = mServoID;
            mConfigPub->publish(configMsg);

            while (mCmdPub->get_subscription_count() == 0) {
                rclcpp::sleep_for(std::chrono::milliseconds(10));
            }
            rclcpp::sleep_for(std::chrono::milliseconds(50));

            // disable torque (control table)
            publishWrite(ADDR_TORQUE_ENABLE, 1, 0);
            rclcpp::sleep_for(std::chrono::milliseconds(10));

            // extended position mode
            publishWrite(ADDR_OPERATING_MODE, 1, 4);

            // write user params
            publishWrite(ADDR_POSITION_P_GAIN, 2, static_cast<uint32_t>(position_p));
            publishWrite(ADDR_POSITION_I_GAIN, 2, static_cast<uint32_t>(position_i));
            publishWrite(ADDR_POSITION_D_GAIN, 2, static_cast<uint32_t>(position_d));
            publishWrite(ADDR_CURRENT_LIMIT, 2, static_cast<uint32_t>(current_limit));
            publishWrite(ADDR_PROFILE_ACCELERATION, 4, static_cast<uint32_t>(profile_acceleration));
            publishWrite(ADDR_PROFILE_VELOCITY, 4, static_cast<uint32_t>(profile_velocity));

            // enable torque (control table)
            publishWrite(ADDR_TORQUE_ENABLE, 1, 1);
        }

        void publishWrite(uint8_t addr, uint8_t len, uint32_t val) {
            msg::ServoIn msg;
            msg.addr = addr;
            msg.length = len;
            msg.value = val;
            mCmdPub->publish(msg);
        }
    };

} // namespace mrover
