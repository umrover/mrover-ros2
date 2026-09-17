#pragma once

#include <algorithm>
#include <format>
#include <functional>
#include <string>
#include <variant>

#include <MRoverCAN.hpp>
#include <moteus/moteus.h>
#include <mrover/msg/can.hpp>
#include <rclcpp/rclcpp.hpp>


namespace mrover {

    using namespace mjbots;

    template<typename Variant, typename NewType>
    struct add_to_variant;

    template<typename... Ts, typename NewType>
    struct add_to_variant<std::variant<Ts...>, NewType> {
        using type = std::variant<Ts..., NewType>;
    };

    using CANMsg_t = add_to_variant<MRoverCANMsg_t, moteus::CanFdFrame>::type;
    static_assert(std::is_trivially_copyable_v<MRoverCANMsg_t> && sizeof(MRoverCANMsg_t) <= 64);

    class CANDevice {
        rclcpp::Node::SharedPtr mNode;
        rclcpp::Publisher<msg::CAN>::SharedPtr mCANPublisher;
        rclcpp::Subscription<msg::CAN>::SharedPtr mCANSubscriber;
        std::string mSrcDevice{}, mDestDevice{};
        std::function<void(CANMsg_t const&)> mCallback;

        void handleIncomingROSMessage(msg::CAN::ConstSharedPtr const& msg) {
            if (!mCallback) return;

            // mask out source
            uint32_t incomingBaseID = msg->prefix & ~CAN_NODE_MASK;

            if (incomingBaseID == MOTEUS_PREFIX) {
                moteus::CanFdFrame frame;
                frame.size = std::min(msg->data.size(), static_cast<size_t>(64));
                std::memcpy(frame.data, msg->data.data(), frame.size);
                mCallback(frame);
                return;
            }

            auto tryParse = [&]<typename T>() {
                if constexpr (!std::is_same_v<T, moteus::CanFdFrame>) {
                    if (T::BASE_ID == incomingBaseID) {
                        mCallback(T{msg->data.data()});
                    }
                }
            };

            [&]<typename... Ts>(std::variant<Ts...>*) {
                (tryParse.operator()<Ts>(), ...);
            }(static_cast<MRoverCANMsg_t*>(nullptr));
        }

    public:
        CANDevice() = default;

        CANDevice(rclcpp::Node::SharedPtr node, std::string srcDevice, std::string destDevice,
                  std::function<void(CANMsg_t const&)> callback = nullptr)
            : mNode{std::move(node)}, mSrcDevice{std::move(srcDevice)},
              mDestDevice{std::move(destDevice)}, mCallback{std::move(callback)} {

            mCANPublisher = mNode->create_publisher<msg::CAN>(std::format("can/{}/out", mDestDevice), 10);
            mCANSubscriber = mNode->create_subscription<msg::CAN>(std::format("can/{}/in", mDestDevice), 10,
                                                                  [this](msg::CAN::ConstSharedPtr const& msg) -> void { handleIncomingROSMessage(msg); });
        }

        void publishMessage(CANMsg_t const& msg, bool const& mjbotsReplyRequest = false) {
            msg::CAN rosMsg;
            rosMsg.source = mSrcDevice;
            rosMsg.destination = mDestDevice;
            rosMsg.mjbots_reply_request = mjbotsReplyRequest;

            std::visit([&](auto const& val) {
                using T = std::decay_t<decltype(val)>;
                if constexpr (std::is_same_v<T, moteus::CanFdFrame>) {
                    rosMsg.prefix = MOTEUS_PREFIX;
                    rosMsg.data.assign(val.data, val.data + val.size);
                } else {
                    rosMsg.prefix = T::BASE_ID;
                    rosMsg.data.assign(val.msg_arr, val.msg_arr + sizeof(val.msg_arr));
                }
            },
                       msg);

            mCANPublisher->publish(rosMsg);
        }
    };
} // namespace mrover
