#include "can_bridge.hpp"

namespace mrover {

    static auto checkSyscallResult(int result) -> int {
        if (result < 0) throw std::system_error{errno, std::generic_category()};

        return result;
    }

    static auto checkErrorCode(boost::system::error_code const& ec) -> void {
        if (ec.value() == boost::system::errc::success) return;

        throw std::runtime_error(std::format("Boost failure: {} {}", ec.value(), ec.message()));
    }

    static auto nearestFittingFdcanFrameSize(std::size_t size) -> std::uint8_t {
        if (size <= 8) return size;
        if (size <= 12) return 12;
        if (size <= 16) return 16;
        if (size <= 20) return 20;
        if (size <= 24) return 24;
        if (size <= 32) return 32;
        if (size <= 48) return 48;
        if (size <= 64) return 64;
        throw std::runtime_error("Too large!");
    }

    CanBridge::CanBridge() : Node{"can_bridge", rclcpp::NodeOptions{}
                                                        .allow_undeclared_parameters(true)
                                                        .automatically_declare_parameters_from_overrides(true)} {
        try {
            RCLCPP_INFO_STREAM(get_logger(), "Starting...");

            // As far as I can tell array of structs in YAML is not even supported by ROS 2 as compared to ROS 1
            // See: https://robotics.stackexchange.com/questions/109909/reading-a-vector-of-structs-as-parameters-in-ros2

            std::map<std::string, rclcpp::Parameter> devices;
            get_parameters("devices", devices);
            mInterface = get_parameter("interface").as_string();

            if (devices.empty()) {
                RCLCPP_FATAL_STREAM(get_logger(), "No devices specified. Did you forget to load the correct ROS parameters?");
                rclcpp::shutdown();
            }

            std::set<std::string> names;
            std::ranges::transform(devices | std::views::keys, std::inserter(names, names.end()), [](std::string const& fullName) {
                return fullName.substr(0, fullName.find('.'));
            });

            for (auto const& name: names) {
                auto id = static_cast<std::uint8_t>(devices.at(std::format("{}.id", name)).as_int());

                mDevices.insert({name, id});

                mDevicesPubSub.emplace(name,
                                       CanFdPubSub{
                                               .publisher = create_publisher<msg::CAN>(std::format("can/{}/in", name), 16),
                                               .subscriber = create_subscription<msg::CAN>(std::format("can/{}/out", name), 16, [this](msg::CAN::ConstSharedPtr const& msg) {
                                                   frameSendRequestCallback(msg);
                                               }),
                                       });

                RCLCPP_INFO_STREAM(get_logger(), std::format("Added device: {} (id: {:#x})", name, id));
            }

            mCanNetLink = CanNetLink{get_logger().get_child("link"), get_parameter("interface").as_string()};

            int socketFileDescriptor = setupSocket();
            mStream.emplace(mIoService);
            mStream->assign(socketFileDescriptor);

            readFrameAsync();

            // Since the constructor needs to return, kick off a self-joining thread to run the IO concurrently
            mIoThread = std::jthread{[this] { mIoService.run(); }};

            RCLCPP_INFO_STREAM(get_logger(), "Started");

        } catch (std::exception const& exception) {
            RCLCPP_FATAL_STREAM(get_logger(), std::format("Failed to start: {}", exception.what()));
            rclcpp::shutdown();
        }
    }

    auto CanBridge::setupSocket() const -> int {
        int socketFd = checkSyscallResult(socket(PF_CAN, SOCK_RAW, CAN_RAW));
        RCLCPP_INFO_STREAM(get_logger(), std::format("Opened CAN socket with file descriptor: {}", socketFd));

        ifreq ifr{};
        std::strcpy(ifr.ifr_name, mInterface.c_str());
        ioctl(socketFd, SIOCGIFINDEX, &ifr);

        sockaddr_can addr{
                .can_family = AF_CAN,
                .can_ifindex = ifr.ifr_ifindex,
        };
        checkSyscallResult(bind(socketFd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)));
        RCLCPP_INFO_STREAM(get_logger(), "Bound CAN socket");

        int enableCanFd = 1;
        checkSyscallResult(setsockopt(socketFd, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enableCanFd, sizeof(enableCanFd)));

        return socketFd;
    }

    auto CanBridge::readFrameAsync() -> void { // NOLINT(*-no-recursion)
        // You would think we would have to read the header first to find the data length (which is not always 64 bytes) and THEN read the data
        // However socketcan is nice and just requires we read the max length
        // It then puts the actual length in the header
        async_read(
                mStream.value(),
                boost::asio::buffer(&mReadFrame, sizeof(mReadFrame)),
                // Supply lambda that is called on completion
                [this](boost::system::error_code const& ec, [[maybe_unused]] std::size_t bytes) { // NOLINT(*-no-recursion)
                    checkErrorCode(ec);
                    assert(bytes == sizeof(mReadFrame));

                    frameReadCallback();

                    // Ready for the next frame, start listening again
                    // Note this is recursive, but it is safe because it is async
                    // i.e. the stack is not growing
                    readFrameAsync();
                });
    }

    auto CanBridge::frameReadCallback() -> void { // NOLINT(*-no-recursion)
        auto [identifier, isErrorFrame, isRemoteTransmissionRequest, isExtendedFrame] = std::bit_cast<RawCanFdId>(mReadFrame.can_id);
        auto [destination, source, isReplyRequired] = std::bit_cast<CanFdMessageId>(static_cast<std::uint16_t>(identifier));

        auto sourceDeviceNameIt = mDevices.right.find(source);
        if (sourceDeviceNameIt == mDevices.right.end()) {
            RCLCPP_WARN_STREAM(get_logger(), std::format("Received message that had an unknown source ID: {}", std::uint8_t{source}));
            return;
        }

        auto destinationDeviceNameIt = mDevices.right.find(destination);
        if (destinationDeviceNameIt == mDevices.right.end()) {
            RCLCPP_WARN_STREAM(get_logger(), std::format("Received message that had an unknown destination ID: {}", std::uint8_t{destination}));
            return;
        }

        msg::CAN msg;
        msg.source = sourceDeviceNameIt->second;
        msg.destination = destinationDeviceNameIt->second;
        msg.data.assign(mReadFrame.data, mReadFrame.data + mReadFrame.len);
        mDevicesPubSub.at(msg.source).publisher->publish(msg);
    }

    auto CanBridge::frameSendRequestCallback(msg::CAN::ConstSharedPtr const& msg) -> void {
        auto sourceIt = mDevices.left.find(msg->source);
        if (sourceIt == mDevices.left.end()) {
            RCLCPP_WARN_STREAM(get_logger(), std::format("Sending message that had an unknown source: {}", msg->source));
            return;
        }

        auto destinationIt = mDevices.left.find(msg->destination);
        if (destinationIt == mDevices.left.end()) {
            RCLCPP_WARN_STREAM(get_logger(), std::format("Sending message that had an unknown destination: {}", msg->destination));
            return;
        }

        CanFdMessageId messageId{
                .destination = destinationIt->second,
                .source = sourceIt->second,
                .replyRequired = static_cast<bool>(msg->reply_required),
        };

        // Craft the SocketCAN frame from the ROS message
        canfd_frame frame{
                .can_id = std::bit_cast<canid_t>(RawCanFdId{
                        .identifier = std::bit_cast<std::uint16_t>(messageId),
                        .isExtendedFrame = get_parameter("is_extended_frame").as_bool(),
                }),
                .len = nearestFittingFdcanFrameSize(msg->data.size()),
        };
        std::memcpy(frame.data, msg->data.data(), msg->data.size());

        try {
            if (std::size_t written = boost::asio::write(mStream.value(), boost::asio::buffer(std::addressof(frame), sizeof(frame)));
                written != sizeof(frame)) {
                RCLCPP_FATAL_STREAM(get_logger(), std::format("Failed to write frame to socket! Expected to write {} bytes, but only wrote {} bytes", sizeof(frame), written));
                rclcpp::shutdown();
            }
        } catch (boost::system::system_error const& error) {
            if (error.code() == boost::asio::error::no_buffer_space) {
                RCLCPP_WARN_STREAM_THROTTLE(get_logger(), *get_clock(), 1000, "No buffer space available to send the message. This usually indicates an electrical problem with the bus. CAN will avoid sending out messages if it can not see other devices.");
                return;
            } else if (error.code() == boost::system::errc::no_such_device_or_address) {
                RCLCPP_WARN(get_logger(), "Device not found.");
            }
            rclcpp::shutdown();
        } catch (...) {
            std::cout << "error" << std::endl;
        }
    }

    CanBridge::~CanBridge() {
        mIoService.stop(); // This causes the io thread to finish
    }

} // namespace mrover

auto main(int argc, char** argv) -> int {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<mrover::CanBridge>());
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
