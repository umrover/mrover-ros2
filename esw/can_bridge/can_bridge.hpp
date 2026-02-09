#pragma once

#include "can_net_link.hpp"
#include "pch.hpp"

namespace mrover {

    static constexpr uint32_t MOTEUS_PREFIX = 0x0000;
    static constexpr uint32_t MOTEUS_REPLY_MASK = 0x8000;

    struct RawCanFdId {
        std::uint32_t identifier : 29 {};
        bool is_error_frame : 1 {};
        bool is_remote_transmission_request : 1 {};
        bool is_extended_frame : 1 {};
    };
    static_assert(sizeof(RawCanFdId) == sizeof(canid_t));

    struct CanFdPubSub {
        rclcpp::Publisher<msg::CAN>::SharedPtr publisher;
        rclcpp::Subscription<msg::CAN>::SharedPtr subscriber;
    };

    class CanBridge : public rclcpp::Node {
        std::string m_interface;
        canfd_frame m_read_frame{};
        CanNetLink m_can_net_link;
        std::optional<boost::asio::posix::basic_stream_descriptor<>> m_stream;
        std::jthread m_io_thread;
        boost::asio::io_service m_io_service;
        boost::bimap<std::string, std::uint8_t> m_devices;
        std::unordered_map<std::string, CanFdPubSub> m_devices_pub_sub;

        [[nodiscard]] auto setup_socket() const -> int;
        void read_frame_async();
        void frame_read_callback();
        void frame_send_request_callback(msg::CAN::ConstSharedPtr const& msg);

    public:
        CanBridge();
        ~CanBridge() override;
    };

} // namespace mrover
