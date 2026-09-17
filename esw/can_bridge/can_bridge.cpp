#include "can_bridge.hpp"

namespace mrover {

    static auto checkSyscallResult(int const result) -> int {
        if (result < 0) throw std::system_error{errno, std::generic_category()};
        return result;
    }

    static auto checkErrorCode(boost::system::error_code const& ec) -> void {
        if (ec.value() == boost::system::errc::success) return;
        throw std::runtime_error(std::format("Boost failure: {} {}", ec.value(), ec.message()));
    }

    static auto nearestFittingFDCANFrameSize(std::size_t const size) -> std::uint8_t {
        if (size <= 8) return size;
        if (size <= 12) return 12;
        if (size <= 16) return 16;
        if (size <= 20) return 20;
        if (size <= 24) return 24;
        if (size <= 32) return 32;
        if (size <= 48) return 48;
        if (size <= 64) return 64;
        throw std::runtime_error("fdcan frame too large");
    }

    CANBridge::CANBridge() : Node{"can_bridge", rclcpp::NodeOptions{}
                                                        .allow_undeclared_parameters(true)
                                                        .automatically_declare_parameters_from_overrides(true)} {
        try {
            RCLCPP_INFO_STREAM(get_logger(), "starting can bridge...");
            std::map<std::string, rclcpp::Parameter> devices;
            get_parameters("devices", devices);
            mInterface = get_parameter("interface").as_string();

            if (devices.empty()) {
                RCLCPP_FATAL_STREAM(get_logger(), "no devices specified - did you forget to load the correct ros params?");
                rclcpp::shutdown();
            }

            std::set<std::string> names;
            std::ranges::transform(devices | std::views::keys, std::inserter(names, names.end()), [](std::string const& fullName) {
                return fullName.substr(0, fullName.find('.'));
            });

            for (auto const& name: names) {
                auto id = static_cast<std::uint8_t>(devices.at(std::format("{}.id", name)).as_int());
                mDevices.insert({name, id});
                mDevicesPubSub.emplace(name, CANFDPubSub{
                                                     .publisher = create_publisher<msg::CAN>(std::format("can/{}/in", name), 16),
                                                     .subscriber = create_subscription<msg::CAN>(std::format("can/{}/out", name), 16,
                                                                                                 [this](msg::CAN::ConstSharedPtr const& msg) { frameSendRequestCallback(msg); }),
                                             });
                RCLCPP_INFO_STREAM(get_logger(), std::format("added device: {} (id: {:#x})", name, id));
            }

            mCANNetLink = CANNetLink{get_logger().get_child("link"), mInterface};
            int socketFD = setupSocket();
            mStream.emplace(mIOService);
            mStream->assign(socketFD);

            readFrameAsync();
            mIOThread = std::jthread{[this] { mIOService.run(); }};
            RCLCPP_INFO_STREAM(get_logger(), "started can bridge");
        } catch (std::exception const& exception) {
            RCLCPP_FATAL_STREAM(get_logger(), std::format("failed: {}", exception.what()));
            rclcpp::shutdown();
        }
    }

    auto CANBridge::setupSocket() const -> int {
        int const socketFD = checkSyscallResult(socket(PF_CAN, SOCK_RAW, CAN_RAW));
        RCLCPP_INFO_STREAM(get_logger(), std::format("opened can socket with file descriptor: {}", socketFD));
        ifreq ifr{};
        std::strcpy(ifr.ifr_name, mInterface.c_str());
        ioctl(socketFD, SIOCGIFINDEX, &ifr);
        sockaddr_can addr{.can_family = AF_CAN, .can_ifindex = ifr.ifr_ifindex};
        checkSyscallResult(bind(socketFD, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)));
        RCLCPP_INFO_STREAM(get_logger(), "bound can socket");
        int const enable_can_fd = 1;
        checkSyscallResult(setsockopt(socketFD, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_can_fd, sizeof(enable_can_fd)));
        return socketFD;
    }

    auto CANBridge::readFrameAsync() -> void {
        async_read(mStream.value(), boost::asio::buffer(&mReadFrame, sizeof(mReadFrame)),
                   [this](boost::system::error_code const& ec, std::size_t const bytes) {
                       checkErrorCode(ec);
                       assert(bytes == sizeof(mReadFrame));
                       frameReadCallback();
                       readFrameAsync();
                   });
    }

    auto CANBridge::frameReadCallback() -> void {
        auto const [identifier, isErr, isRTR, isExt] = std::bit_cast<RawCANFDID_t>(mReadFrame.can_id);

        // extract prefix, source, and dest
        uint32_t const destID = (identifier & CAN_DEST_ID_MASK) >> CAN_DEST_ID_OFFSET;
        uint32_t const srcID = (identifier & CAN_SRC_ID_MASK) >> CAN_SRC_ID_OFFSET;
        uint32_t const prefix = identifier & ~CAN_NODE_MASK;

        auto const srcIterator = mDevices.right.find(srcID);
        auto const destIterator = mDevices.right.find(destID);

        // drop any packets with invalid src/dest registers
        if (srcIterator == mDevices.right.end() || destIterator == mDevices.right.end()) {
            RCLCPP_INFO(get_logger(), "could not find one of %x or %x", srcID, destID);
            return;
        }

        // create ros2 canfd frame
        msg::CAN msg;
        msg.source = srcIterator->second;
        msg.destination = destIterator->second;
        msg.prefix = prefix;
        msg.data.assign(mReadFrame.data, mReadFrame.data + mReadFrame.len);

        // publish frame
        mDevicesPubSub.at(srcIterator->second).publisher->publish(msg);
    }

    auto CANBridge::frameSendRequestCallback(msg::CAN::ConstSharedPtr const& msg) -> void {
        auto const srcIterator = mDevices.left.find(msg->source);
        if (srcIterator == mDevices.left.end()) {
            RCLCPP_WARN_STREAM(get_logger(), std::format("sending message that has an unknown source: {}", msg->source));
            return;
        }

        auto const destIterator = mDevices.left.find(msg->destination);
        if (destIterator == mDevices.left.end()) {
            RCLCPP_WARN_STREAM(get_logger(), std::format("sending message that has an unknown destination: {}", msg->destination));
            return;
        }

        if (srcIterator == mDevices.left.end() || destIterator == mDevices.left.end()) return;

        uint32_t idBits = (static_cast<uint32_t>(msg->prefix) & ~CAN_NODE_MASK) // no shifts here, 16 LSBs of prefix are 0
                          | ((static_cast<std::uint32_t>(srcIterator->second) << CAN_SRC_ID_OFFSET) & CAN_SRC_ID_MASK) | ((static_cast<std::uint32_t>(destIterator->second) << CAN_DEST_ID_OFFSET) & CAN_DEST_ID_MASK);

        // put in reply req bit for moteus
        if (msg->prefix == MOTEUS_PREFIX && msg->mjbots_reply_request) {
            idBits |= MOTEUS_REPLY_MASK;
        }

        canfd_frame frame{
                .can_id = std::bit_cast<canid_t>(RawCANFDID_t{
                        .identifier = std::bit_cast<uint32_t>(idBits),
                        .is_extended_frame = true}),
                .len = nearestFittingFDCANFrameSize(msg->data.size())};
        std::memcpy(frame.data, msg->data.data(), std::min(msg->data.size(), static_cast<size_t>(64)));

        try {
            if (std::size_t written = boost::asio::write(mStream.value(), boost::asio::buffer(std::addressof(frame), sizeof(frame)));
                written != sizeof(frame)) {
                RCLCPP_FATAL_STREAM(get_logger(), std::format("failed to write frame to socket: expected to write {} bytes, but only wrote {} bytes", sizeof(frame), written));
                rclcpp::shutdown();
            }
        } catch (boost::system::system_error const& error) {
            if (error.code() == boost::asio::error::no_buffer_space) {
                RCLCPP_WARN_STREAM_THROTTLE(get_logger(), *get_clock(), 1000, "no buffer space available to send the message - this usually indicates an electrical problem with the bus, as can will avoid sending out messages if it can not see other devices.");
                return;
            }
            if (error.code() == boost::system::errc::no_such_device_or_address) {
                RCLCPP_WARN(get_logger(), "device not found");
            }
            rclcpp::shutdown();
        } catch (...) {
            std::cout << "error" << std::endl;
        }
    }

    CANBridge::~CANBridge() { mIOService.stop(); }

} // namespace mrover


auto main(int const argc, char** argv) -> int {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<mrover::CANBridge>());
    rclcpp::shutdown();
    return 0;
}
