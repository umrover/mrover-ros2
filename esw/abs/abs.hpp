#pragma once

#include "MRoverCAN.hpp"
#include "can_device.hpp"
#include <rclcpp/rclcpp.hpp>
#include <units.hpp>

namespace mrover {

    class AbsoluteEncoder {
        rclcpp::Node::SharedPtr mNode;
        CANDevice mDevice;

        Radians mPosition{0.0f};
        RadiansPerSecond mVelocity{0.0f};
    public:
        AbsoluteEncoder(rclcpp::Node::SharedPtr node, std::string masterName, std::string deviceName)
            : mNode{std::move(node)},
              mDevice{mNode, std::move(masterName), std::move(deviceName),
                      [this](CANMsg_t const& msg) -> void { processMessage(msg); }} {

        }

        void processMessage(CANMsg_t const& msg) {
            std::visit([this](auto const& decoded) -> void {
                using T = std::decay_t<decltype(decoded)>;

                if constexpr (std::is_same_v<T, ABSEncoderState>) {
                    // update local state
                    mPosition = Radians{decoded.position};
                    mVelocity = RadiansPerSecond{decoded.velocity};
                } else {
                    RCLCPP_WARN(mNode->get_logger(), "absolute encoder received unexpected message type %s", typeid(T).name());
                }
            },
                       msg);
        }

        [[nodiscard]] auto getPosition() const -> Radians {
            return mPosition;
        }

        [[nodiscard]] auto getVelocity() const -> RadiansPerSecond{
            return mVelocity;
        }
    };

} // namespace mrover
