#pragma once

#include <algorithm>
#include <format>
#include <functional>
#include <string>
#include <variant>

#include <CANBus1.hpp>
#include <moteus/moteus.h>
#include <mrover/msg/can.hpp>
#include <rclcpp/rclcpp.hpp>


namespace mrover {

    using namespace mjbots;

    static constexpr uint32_t MOTEUS_PREFIX = 0x0000;

    template<typename Variant, typename NewType>
    struct add_to_variant;

    template<typename... Ts, typename NewType>
    struct add_to_variant<std::variant<Ts...>, NewType> {
        using type = std::variant<Ts..., NewType>;
    };

    using can_msg_t = add_to_variant<CANBus1Msg_t, moteus::CanFdFrame>::type;

    class CanDevice {
        rclcpp::Node::SharedPtr m_node;
        rclcpp::Publisher<msg::CAN>::SharedPtr m_can_publisher;
        rclcpp::Subscription<msg::CAN>::SharedPtr m_can_subscriber;
        std::string m_src_device{}, m_dest_device{};
        std::function<void(can_msg_t const&)> m_callback;

        void handle_incoming_ros(msg::CAN::ConstSharedPtr const& msg) {
            if (!m_callback) return;

            // mask out source
            uint32_t incoming_base_id = msg->prefix & ~CAN_NODE_MASK;

            if (incoming_base_id == MOTEUS_PREFIX) {
                moteus::CanFdFrame frame;
                frame.size = std::min(msg->data.size(), static_cast<size_t>(64));
                std::memcpy(frame.data, msg->data.data(), frame.size);
                m_callback(frame);
                return;
            }

            auto try_parse = [&]<typename T>() {
                if constexpr (!std::is_same_v<T, moteus::CanFdFrame>) {
                    if (T::BASE_ID == incoming_base_id) {
                        m_callback(T{msg->data.data()});
                    }
                }
            };

            [&]<typename... Ts>(std::variant<Ts...>*) {
                (try_parse.operator()<Ts>(), ...);
            }(static_cast<CANBus1Msg_t*>(nullptr));
        }

    public:
        CanDevice() = default;

        CanDevice(rclcpp::Node::SharedPtr node, std::string src_device, std::string dest_device,
                  std::function<void(can_msg_t const&)> callback = nullptr)
            : m_node{std::move(node)}, m_src_device{std::move(src_device)},
              m_dest_device{std::move(dest_device)}, m_callback{std::move(callback)} {

            m_can_publisher = m_node->create_publisher<msg::CAN>(std::format("can/{}/out", m_dest_device), 10);
            m_can_subscriber = m_node->create_subscription<msg::CAN>(std::format("can/{}/in", m_dest_device), 10,
                                                                     [this](msg::CAN::ConstSharedPtr const& msg) -> void { handle_incoming_ros(msg); });
        }

        void publish_message(can_msg_t const& msg, bool const& mjbots_reply_request = false) {
            msg::CAN ros_msg;
            ros_msg.source = m_src_device;
            ros_msg.destination = m_dest_device;
            ros_msg.mjbots_reply_request = mjbots_reply_request;

            std::visit([&](auto const& val) {
                using T = std::decay_t<decltype(val)>;
                if constexpr (std::is_same_v<T, moteus::CanFdFrame>) {
                    ros_msg.prefix = MOTEUS_PREFIX;
                    ros_msg.data.assign(val.data, val.data + val.size);
                } else {
                    ros_msg.prefix = T::BASE_ID;
                    ros_msg.data.assign(val.msg_arr, val.msg_arr + sizeof(val.msg_arr));
                }
            },
                       msg);

            m_can_publisher->publish(ros_msg);
        }
    };
} // namespace mrover
