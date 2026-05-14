#pragma once

#include "MRoverCAN.hpp"
#include "can_device.hpp"
#include <rclcpp/rclcpp.hpp>
#include <units.hpp>

namespace mrover {

    class LimitSwitchBoard {
        rclcpp::Node::SharedPtr mNode;
        CANDevice mDevice;

        bool mLimitAPressed{};
        bool mLimitBPressed{};

    public:
        LimitSwitchBoard(rclcpp::Node::SharedPtr node, std::string masterName, std::string deviceName)
            : mNode{std::move(node)},
              mDevice{mNode, std::move(masterName), std::move(deviceName),
                      [this](CANMsg_t const& msg) -> void { processMessage(msg); }} {
        }

        void processMessage(CANMsg_t const& msg) {
            std::visit([this](auto const& decoded) -> void {
                using T = std::decay_t<decltype(decoded)>;

                if constexpr (std::is_same_v<T, LIMState>) {
                    // update local state
                    mLimitAPressed = decoded.lim_a;
                    mLimitBPressed = decoded.lim_b;
                } else {
                    RCLCPP_WARN(mNode->get_logger(), "limit switch board received unexpected message type %s", typeid(T).name());
                }
            },
                       msg);
        }

        [[nodiscard]] auto getLimitA() const -> bool {
            return mLimitAPressed;
        }

        [[nodiscard]] auto getLimitB() const -> bool {
            return mLimitBPressed;
        }
    };

} // namespace mrover
