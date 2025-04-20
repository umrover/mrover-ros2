#include "gst_v4l2_encoder.hpp"

namespace mrover {

    using namespace std::string_view_literals;

    template<typename T>
    auto gstCheck(T* t) -> T* {
        if (!t) throw std::runtime_error{"Failed to create"};
        return t;
    }

    auto gstCheck(gboolean b) -> void {
        if (!b) throw std::runtime_error{"Failed to create"};
    }

    auto gstBusMessage(GstBus*, GstMessage* message, gpointer data) -> gboolean;

    auto makeRTPH264SinkString(std::string_view host, std::uint16_t port) -> std::string {
        return std::format("! rtph264pay ! udpsink host={} port={}", host, port);
    }

    auto makeRTPH265SinkString(std::string_view host, std::uint16_t port) -> std::string {
        return std::format("! rtph265pay ! udpsink host={} port={}", host, port);
    }

    auto GstV4L2Encoder::initPipeline(std::string_view deviceNode) -> void {
        RCLCPP_INFO_STREAM(get_logger(), "Initializing and starting GStreamer pipeline...");

        mMainLoop = gstCheck(g_main_loop_new(nullptr, FALSE));

        // Source
        std::string launch;
        std::string captureFormat;
        if (deviceNode.empty()) {
            launch += "videotestsrc ";
            captureFormat = "video/x-raw";
        } else {
            launch += std::format("v4l2src device={} ", deviceNode);
            captureFormat = mDecodeJpegFromDevice ? "image/jpeg" : "video/x-raw,format=YUY2";
            if (mDisableAutoWhiteBalance) launch += "extra-controls=\"c,white_balance_temperature_auto=0,white_balance_temperature=4000\" ";
        }
        // Source format
        launch += std::format("! {},width={},height={},framerate={}/1 ", captureFormat, mImageWidth, mImageHeight, mImageFramerate);

        // Source decoder and H265 encoder
        if (gst_element_factory_find("nvv4l2h265enc")) {
            // Most likely on the Jetson
            if (captureFormat.contains("jpeg")) {
                // Mostly used with USB cameras, MPEG capture uses way less USB bandwidth
                launch +=
                        // TODO(quintin): I had to apply this patch: https://forums.developer.nvidia.com/t/macrosilicon-usb/157777/4
                        //                nvv4l2camerasrc only supports UYUV by default, but our cameras are YUY2 (YUYV)
                        // "nvv4l2camerasrc device={} "
                        // "! video/x-raw(memory:NVMM),format=YUY2,width={},height={},framerate={}/1 "

                        "! nvv4l2decoder mjpeg=1 " // Hardware-accelerated JPEG decoding, output is apparently some unknown proprietary format
                        "! nvvidconv "             // Convert from proprietary format to NV12 so the encoder understands it
                        "! video/x-raw(memory:NVMM),format=NV12 ";
            } else {
                launch += "! videoconvert " // Convert to I420 for the encoder, note we are still on the CPU
                          "! video/x-raw,format=I420 "
                          "! nvvidconv " // Upload to GPU memory for the encoder
                          "! video/x-raw(memory:NVMM),format=I420 ";
            }
            launch += std::format("! nvv4l2h265enc name=encoder bitrate={} iframeinterval=300 vbv-size=33333 insert-sps-pps=true control-rate=constant_bitrate profile=Main num-B-Frames=0 ratecontrol-enable=true preset-level=UltraFastPreset EnableTwopassCBR=false maxperf-enable=true ",
                                  mBitrate);
        } else if (gst_element_factory_find("nvh265enc")) {
            // For desktop/laptops with the custom NVIDIA bad gstreamer plugins built (a massive pain to do!)
            if (captureFormat.contains("jpeg")) {
                launch += "! jpegdec ";
            } else {
                launch += "! videoconvert ";
            }
            launch += "! nvh265enc name=encoder ";
        } else {
            // For desktop/laptops with no hardware encoder
            if (captureFormat.contains("jpeg")) {
                launch += "! jpegdec ";
            } else {
                launch += "! videoconvert ";
            }
            launch += std::format("! x264enc tune=zerolatency bitrate={} name=encoder ", mBitrate);
        }
        if (launch.contains("264")) {
            launch += makeRTPH264SinkString(mAddress, mPort);
        } else if (launch.contains("265")) {
            launch += makeRTPH265SinkString(mAddress, mPort);
        } else {
            throw std::runtime_error{"Unsupported codec"};
        }

        RCLCPP_INFO_STREAM(get_logger(), std::format("GStreamer launch string: {}", launch));
        mPipeline = gstCheck(gst_parse_launch(launch.c_str(), nullptr));

        GstBus* bus = gstCheck(gst_element_get_bus(mPipeline));
        gst_bus_add_watch(bus, gstBusMessage, this);
        gst_object_unref(bus);

        if (gst_element_set_state(mPipeline, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE){
            throw std::runtime_error{"Failed to play GStreamer pipeline"};
        }


        mMainLoopThread = std::thread{[this] {
            RCLCPP_INFO_STREAM(get_logger(), "Entering GStreamer main loop");
            g_main_loop_run(mMainLoop);
            RCLCPP_INFO_STREAM(get_logger(), "Leaving GStreamer main loop");
        }};

        RCLCPP_INFO_STREAM(get_logger(), "Initialized and started GStreamer pipeline");
    }

    auto findDeviceNode(std::string_view devicePath) -> std::string {
        udev* udevContext = udev_new();
        if (!udevContext) throw std::runtime_error{"Failed to initialize udev"};

        udev_enumerate* enumerate = udev_enumerate_new(udevContext);
        if (!enumerate) throw std::runtime_error{"Failed to create udev enumeration"};

        udev_enumerate_add_match_subsystem(enumerate, "video4linux");
        udev_enumerate_scan_devices(enumerate);

        udev_list_entry* devices = udev_enumerate_get_list_entry(enumerate);
        if (!devices) throw std::runtime_error{"Failed to get udev device list"};

        udev_device* device{};
        udev_list_entry* entry;
        udev_list_entry_foreach(entry, devices) {
            udev_device* candidateDevice = udev_device_new_from_syspath(udevContext, udev_list_entry_get_name(entry));
            if (!candidateDevice) throw std::runtime_error{"Failed to get udev device"};

            std::string candidateDevicePath = udev_device_get_devpath(candidateDevice);

            if (!candidateDevicePath.starts_with(devicePath)) continue;

            device = candidateDevice;
            break;
        }
        if (!device) throw std::runtime_error{"Failed to find udev device"};

        std::string deviceNode = udev_device_get_devnode(device);

        udev_device_unref(device);
        udev_enumerate_unref(enumerate);
        udev_unref(udevContext);

        return deviceNode;
    }

    GstV4L2Encoder::GstV4L2Encoder([[maybe_unused]] rclcpp::NodeOptions const& options) : Node{"gst_v4l2_encoder", rclcpp::NodeOptions{}.use_intra_process_comms(true)} {
        try {
            declare_parameter("dev_node", "");
            declare_parameter("dev_path", "");
            declare_parameter("decode_jpeg_from_device", rclcpp::ParameterType::PARAMETER_BOOL);
            declare_parameter("disable_auto_white_balance", rclcpp::ParameterType::PARAMETER_BOOL);
            declare_parameter("width", rclcpp::ParameterType::PARAMETER_INTEGER);
            declare_parameter("height", rclcpp::ParameterType::PARAMETER_INTEGER);
            declare_parameter("framerate", rclcpp::ParameterType::PARAMETER_INTEGER);
            declare_parameter("address", rclcpp::ParameterType::PARAMETER_STRING);
            declare_parameter("port", rclcpp::ParameterType::PARAMETER_INTEGER);
            declare_parameter("bitrate", rclcpp::ParameterType::PARAMETER_INTEGER);

            mDecodeJpegFromDevice = get_parameter("decode_jpeg_from_device").as_bool();
            mImageWidth = get_parameter("width").as_int();
            mImageHeight = get_parameter("height").as_int();
            mImageFramerate = get_parameter("framerate").as_int();
            mAddress = get_parameter("address").as_string();
            mPort = static_cast<std::uint16_t>(get_parameter("port").as_int());
            mBitrate = get_parameter("bitrate").as_int();
            // For example, /dev/video0
            // These device paths are not garunteed to stay the same between reboots
            // Prefer sys path for non-debugging purposes
            std::string deviceNode = get_parameter("dev_node").as_string();
            // To find the sys path:
            // 1) Disconnect all cameras
            // 2) Confirm there are no /dev/video* devices
            // 2) Connect the camera you want to use
            // 3) Run "ls /dev/video*" to verify the device is connected
            // 4) Run "udevadm info -q path -n /dev/video0" to get the sys path
            std::string devicePath = get_parameter("dev_path").as_string();

            if (!devicePath.empty()) {
                deviceNode = findDeviceNode(devicePath);
            }

            gst_init(nullptr, nullptr);

            initPipeline(deviceNode);


        } catch (std::exception const& e) {
            RCLCPP_ERROR_STREAM(get_logger(), std::format("Exception initializing GStreamer V4L2 streamer: {}", e.what()));
            rclcpp::shutdown();
        }
    }

    GstV4L2Encoder::~GstV4L2Encoder() {
        if (mMainLoop) {
            g_main_loop_quit(mMainLoop);
            mMainLoopThread.join();
            g_main_loop_unref(mMainLoop);
        }

        if (mPipeline) {
            gst_element_set_state(mPipeline, GST_STATE_NULL);
            gst_object_unref(mPipeline);
        }
    }

    auto gstBusMessage(GstBus*, GstMessage* message, gpointer data) -> gboolean {
        auto node = static_cast<GstV4L2Encoder*>(data);
        switch (message->type) {
            case GST_MESSAGE_INFO:
            case GST_MESSAGE_WARNING:
            case GST_MESSAGE_ERROR: {
                GError* error;
                gchar* debug;
                auto logger = node->get_logger().get_child("gstreamer");
                switch (message->type) {
                    case GST_MESSAGE_INFO:
                        gst_message_parse_info(message, &error, &debug);
                        RCLCPP_INFO_STREAM(logger, std::format("{} ({})", error->message, debug));
                        break;
                    case GST_MESSAGE_WARNING:
                        gst_message_parse_warning(message, &error, &debug);
                        RCLCPP_WARN_STREAM(logger, std::format("{} ({})", error->message, debug));
                        break;
                    case GST_MESSAGE_ERROR:
                        gst_message_parse_error(message, &error, &debug);
                        RCLCPP_FATAL_STREAM(logger, std::format("{} ({})", error->message, debug));
                        rclcpp::shutdown();
                        break;
                    default:
                        std::abort();
                }
                g_error_free(error);
                g_free(debug);
                break;
            }
            case GST_MESSAGE_EOS: {
                RCLCPP_ERROR_STREAM(node->get_logger().get_child("gstreamer"), "End of stream");
                rclcpp::shutdown();
                break;
            }
            default:
                break;
        }
        return TRUE;
    }

} // namespace mrover


#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(mrover::GstV4L2Encoder)
