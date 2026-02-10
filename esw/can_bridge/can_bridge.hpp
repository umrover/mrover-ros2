#pragma once

#include "can_net_link.hpp"
#include "pch.hpp"

namespace mrover {

    static constexpr uint32_t MOTEUS_PREFIX = 0x0000;
    static constexpr uint32_t MOTEUS_REPLY_MASK = 0x8000;

    struct RawCANFDID_t {
        std::uint32_t identifier : 29 {};
        bool is_error_frame : 1 {};
        bool is_remote_transmission_request : 1 {};
        bool is_extended_frame : 1 {};
    };
    static_assert(sizeof(RawCANFDID_t) == sizeof(canid_t));

    struct CANFDPubSub {
        rclcpp::Publisher<msg::CAN>::SharedPtr publisher;
        rclcpp::Subscription<msg::CAN>::SharedPtr subscriber;
    };

    class CANBridge : public rclcpp::Node {
        std::string mInterface;
        canfd_frame mReadFrame{};
        CanNetLink mCANNetLink;
        std::optional<boost::asio::posix::basic_stream_descriptor<>> mStream;
        std::jthread mIOThread;
        boost::asio::io_service mIOService;
        boost::bimap<std::string, std::uint8_t> mDevices;
        std::unordered_map<std::string, CANFDPubSub> mDevicesPubSub;

        [[nodiscard]] auto setupSocket() const -> int;
        void readFrameAsync();
        void frameReadCallback();
        void frameSendRequestCallback(msg::CAN::ConstSharedPtr const& msg);

    public:
        CANBridge();
        ~CANBridge() override;
    };

} // namespace mrover
