#pragma once

#include "can_device.hpp"
#include "units.hpp"
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <variant>

namespace mrover {
    static constexpr std::size_t NUM_LIMITS = 4;

    template<typename Derived>
    class ControllerBase {
    public:
        ControllerBase(rclcpp::Node::SharedPtr node, std::string masterName, std::string controllerName)
            : mNode{std::move(node)},
              mMasterName{std::move(masterName)},
              mControllerName{std::move(controllerName)},
              mDevice{mNode, mMasterName, mControllerName,
                      [this](CANMsg_t const& msg) {
                          static_cast<Derived*>(this)->processMessage(msg);
                      }} {}

        virtual ~ControllerBase() = default;

        // common state accessors
        [[nodiscard]] auto getName() const -> std::string { return mControllerName; }
        [[nodiscard]] auto getState() const -> std::string { return mState; }
        [[nodiscard]] auto getError() const -> std::string { return mErrorState; }
        [[nodiscard]] auto getPosition() const -> float { return mPosition; }
        [[nodiscard]] auto getVelocity() const -> float { return mVelocity; }
        [[nodiscard]] auto getCurrent() const -> float { return mCurrent; }
        [[nodiscard]] auto getIsStalled() const -> bool { return mIsStalled; }
        [[nodiscard]] auto getLimitsHitBits() const -> std::uint8_t {
            std::uint8_t limits_hit{};
            for (std::size_t i = 0; i < NUM_LIMITS; ++i) {
                limits_hit |= mLimitHit.at(i) << i;
            }
            return limits_hit;
        }

    protected:
        rclcpp::Node::SharedPtr mNode;
        std::string mMasterName, mControllerName;
        CANDevice mDevice;

        // state variables
        std::string mState{"Stopped"};
        std::string mErrorState{"None"};
        float mPosition{0.0f};
        float mVelocity{0.0f};
        float mCurrent{0.0f};
        std::array<bool, NUM_LIMITS> mLimitHit{};
        bool mIsStalled{false};
    };

} // namespace mrover
