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

    auto GstV4L2Encoder::initPipeline(std::string_view deviceNode) -> void {
        RCLCPP_INFO_STREAM(get_logger(), "Initializing and starting GStreamer pipeline...");

        mMainLoop = gstCheck(g_main_loop_new(nullptr, FALSE));

        // Source
        std::string launch;
        std::string captureFormat;
        if (deviceNode.empty()) {
            launch += "videotestsrc ! video/x-raw";
            launch += std::format(" ! {},width={},height={},framerate={}/1", captureFormat, mImageWidth, mImageHeight, mImageFramerate);
        } else {
            launch += gst::Video::V4L2::createSrc(deviceNode,
                                                  mDecodeJpegFromDevice ? gst::Video::V4L2::Format::MJPG : gst::Video::V4L2::Format::YUYV,
                                                  mImageWidth, mImageHeight, mImageFramerate);

            // gst::Video::V4L2::addProperty("extra-controls", "\"c,white_balance_temperature_auto=0,white_balance_temperature=4000\"")
            // if (mDisableAutoWhiteBalance) launch += "extra-controls=\"c,white_balance_temperature_auto=0,white_balance_temperature=4000\" ";
        }
        // Source format

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
            launch += std::format("! videocrop left={} right={} top={} bottom={} ", mCropLeft, mCropRight, mCropTop, mCropBottom);
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
            launch += gst::Video::createRtpSink(mAddress, mPort, gst::Video::Codec::H264);
        } else if (launch.contains("265")) {
            launch += gst::Video::createRtpSink(mAddress, mPort, gst::Video::Codec::H265);
        } else {
            throw std::runtime_error{"Unsupported codec"};
        }

        RCLCPP_INFO_STREAM(get_logger(), std::format("GStreamer launch string: {}", launch));
        mPipeline = gstCheck(gst_parse_launch(launch.c_str(), nullptr));

        GstBus* bus = gstCheck(gst_element_get_bus(mPipeline));
        gst_bus_add_watch(bus, gstBusMessage, this);
        gst_object_unref(bus);

        if (gst_element_set_state(mPipeline, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
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
            declare_parameter("camera", rclcpp::ParameterType::PARAMETER_STRING);
            std::string const cameraName = get_parameter("camera").as_string();

            std::string const addressParamName = std::format("{}.address", cameraName);
            declare_parameter(addressParamName, rclcpp::ParameterType::PARAMETER_STRING);
            mAddress = get_parameter(addressParamName).as_string();

            std::string const portParamName = std::format("{}.port", cameraName);
            declare_parameter(portParamName, rclcpp::ParameterType::PARAMETER_INTEGER);
            mPort = static_cast<std::uint16_t>(get_parameter(portParamName).as_int());

            // For example, /dev/video0
            // These device paths are not garunteed to stay the same between reboots
            // Prefer sys path for non-debugging purposes
            std::string const deviceNodeParamName = std::format("{}.dev_node", cameraName);
            declare_parameter(deviceNodeParamName, "");
            std::string deviceNode = get_parameter(deviceNodeParamName).as_string();

            // To find the sys path:
            // 1) Disconnect all cameras
            // 2) Confirm there are no /dev/video* devices
            // 2) Connect the camera you want to use
            // 3) Run "ls /dev/video*" to verify the device is connected
            // 4) Run "udevadm info -q path -n /dev/video0" to get the sys path
            std::string const devicePathParamName = std::format("{}.dev_path", cameraName);
            declare_parameter(devicePathParamName, "");
            std::string devicePath = get_parameter(devicePathParamName).as_string();

            std::string const decodeJpegFromDeviceParamName = std::format("{}.decode_jpeg_from_device", cameraName);
            declare_parameter(decodeJpegFromDeviceParamName, rclcpp::ParameterType::PARAMETER_BOOL);
            mDecodeJpegFromDevice = get_parameter(decodeJpegFromDeviceParamName).as_bool();

            declare_parameter("disable_auto_white_balance", rclcpp::ParameterType::PARAMETER_BOOL);

            std::string const widthParamName = std::format("{}.width", cameraName);
            declare_parameter(widthParamName, rclcpp::ParameterType::PARAMETER_INTEGER);
            mImageWidth = get_parameter(widthParamName).as_int();

            std::string const heightParamName = std::format("{}.height", cameraName);
            declare_parameter(heightParamName, rclcpp::ParameterType::PARAMETER_INTEGER);
            mImageHeight = get_parameter(heightParamName).as_int();

            std::string const framerateParamName = std::format("{}.framerate", cameraName);
            declare_parameter(framerateParamName, rclcpp::ParameterType::PARAMETER_INTEGER);
            mImageFramerate = get_parameter(framerateParamName).as_int();

            std::string const bitrateParamName = std::format("{}.bitrate", cameraName);
            declare_parameter(bitrateParamName, rclcpp::ParameterType::PARAMETER_INTEGER);
            mBitrate = get_parameter(bitrateParamName).as_int();

            declare_parameter(std::format("{}.crop_left", cameraName), 0);
            declare_parameter(std::format("{}.crop_right", cameraName), 0);
            declare_parameter(std::format("{}.crop_top", cameraName), 0);
            declare_parameter(std::format("{}.crop_bottom", cameraName), 0);
            mCropLeft = static_cast<int>(get_parameter(std::format("{}.crop_left", cameraName)).as_int());
            mCropRight = static_cast<int>(get_parameter(std::format("{}.crop_right", cameraName)).as_int());
            mCropTop = static_cast<int>(get_parameter(std::format("{}.crop_top", cameraName)).as_int());
            mCropBottom = static_cast<int>(get_parameter(std::format("{}.crop_bottom", cameraName)).as_int());

            mCropEnabled = mCropLeft != 0 || mCropRight != 0 || mCropTop != 0 || mCropBottom != 0;

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
