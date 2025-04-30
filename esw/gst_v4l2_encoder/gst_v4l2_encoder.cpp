#include "gst_v4l2_encoder.hpp"
#include "gst.hpp"

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

    auto GstV4L2Encoder::createLaunchString(std::string_view deviceNode) -> void {
        gst::PipelineBuilder pipeline;
        // Source
        if (deviceNode.empty()) {
            mCaptureFormat = gst::video::v4l2::Format::YUYV;
            pipeline.pushBack("videotestsrc");
            pipeline.pushBack(std::format("video/x-raw,format={},width={},height={},framerate={}/1", gst::video::v4l2::toStringGstType(mCaptureFormat), mImageWidth, mImageHeight, mImageFramerate));
        } else {
            pipeline.pushBack(gst::video::v4l2::createSrc(deviceNode, mCaptureFormat, mImageWidth, mImageHeight, mImageFramerate));
            // gst::video::v4l2::addProperty("extra-controls", "\"c,white_balance_temperature_auto=0,white_balance_temperature=4000\"")
            // if (mDisableAutoWhiteBalance) mLaunch += "extra-controls=\"c,white_balance_temperature_auto=0,white_balance_temperature=4000\" ";
        }

        if (gst::video::v4l2::isRawFormat(mCaptureFormat) && mCropEnabled) {
            pipeline.pushBack(std::format("videocrop left={} right={} top={} bottom={}", mCropLeft, mCropRight, mCropTop, mCropBottom));
        }

        // Source decoder and H265 encoder
        if (gst_element_factory_find("nvv4l2h265enc")) {
            // Most likely on the Jetson
            if (mCaptureFormat == gst::video::v4l2::Format::MJPG) {
                // TODO(quintin): I had to apply this patch: https://forums.developer.nvidia.com/t/macrosilicon-usb/157777/4
                //                nvv4l2camerasrc only supports UYUV by default, but our cameras are YUY2 (YUYV)
                // "nvv4l2camerasrc device={} "
                // "! video/x-raw(memory:NVMM),format=YUY2,width={},height={},framerate={}/1 "

                // Mostly used with USB cameras, MPEG capture uses way less USB bandwidth
                pipeline.pushBack("nvv4l2decoder", gst::addProperty("mjpeg", 1)); // Hardware-accelerated JPEG decoding, output is apparently some unknown proprietary format
                pipeline.pushBack("nvvidconv");                                   // Convert from proprietary format to NV12 so the encoder understands it
                pipeline.pushBack("video/x-raw(memory:NVMM),format=NV12");

            } else {
                pipeline.pushBack("videoconvert");
                pipeline.pushBack("video/x-raw,format=I420"); // Convert to I420 for the encoder, note we are still on the CPU
                pipeline.pushBack("nvvidconv");               // Upload to GPU memory for the encoder
                pipeline.pushBack("video/x-raw(memory:NVMM),format=I420");
            }

            pipeline.pushBack("nvv4l2h265enc",
                              gst::addProperty("bitrate", mBitrate),
                              gst::addProperty("iframeinterval", 300),
                              gst::addProperty("vbv-size", 33333),
                              gst::addProperty("insert-sps-pps", true),
                              gst::addProperty("control-rate", "constant_bitrate"),
                              gst::addProperty("profile", "Main"),
                              gst::addProperty("num-B-Frames", 0),
                              gst::addProperty("ratecontrol-enable", true),
                              gst::addProperty("preset-level", "UltraFastPreset"),
                              gst::addProperty("EnableTwopassCBR", false),
                              gst::addProperty("maxperf-enable", true));
        } else {
            // For desktop/laptops with no hardware encoder
            if (gst::video::v4l2::isCompressedFormat(mCaptureFormat)) {
                pipeline.pushBack(gst::video::createDefaultDecoder(toStringGstType(mCaptureFormat)));
            } else {
                pipeline.pushBack("videoconvert");
            }

            pipeline.pushBack(gst::video::createDefaultEncoder(mStreamCodec));

            if (mStreamCodec == gst::video::Codec::H264) {
                pipeline.addPropsToElement(pipeline.size() - 1,
                                           gst::addProperty("tune", "zerolatency"),
                                           gst::addProperty("bitrate", mBitrate),
                                           gst::addProperty("name", "encoder"));
            }
        }

        pipeline.pushBack(gst::video::createRtpSink(mAddress, mPort, mStreamCodec));

        mLaunch = pipeline.str();
    }

    auto GstV4L2Encoder::initPipeline() -> void {
        RCLCPP_INFO_STREAM(get_logger(), "Initializing and starting GStreamer pipeline...");

        mMainLoop = gstCheck(g_main_loop_new(nullptr, FALSE));
        RCLCPP_INFO_STREAM(get_logger(), std::format("GStreamer launch string: {}", mLaunch));
        mPipeline = gstCheck(gst_parse_launch(mLaunch.c_str(), nullptr));

        GstBus* bus = gstCheck(gst_element_get_bus(mPipeline));
        gst_bus_add_watch(bus, gstBusMessage, this);
        gst_object_unref(bus);

        mMainLoopThread = std::thread{[this] {
            RCLCPP_INFO_STREAM(get_logger(), "Entering GStreamer main loop");
            g_main_loop_run(mMainLoop);
            RCLCPP_INFO_STREAM(get_logger(), "Leaving GStreamer main loop");
        }};

        RCLCPP_INFO_STREAM(get_logger(), "Initialized and started GStreamer pipeline");
    }

    auto GstV4L2Encoder::playPipeline() -> void {
        if (gst_element_set_state(mPipeline, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
            throw std::runtime_error{"Failed to play GStreamer pipeline"};
        }
    }

    auto GstV4L2Encoder::pausePipeline() -> void {
        if (gst_element_set_state(mPipeline, GST_STATE_PAUSED) == GST_STATE_CHANGE_FAILURE) {
            throw std::runtime_error{"Failed to pause GStreamer pipeline"};
        }
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

            int port;
            std::string captureFormat, codec;
            // For example, /dev/video0
            // These device paths are not garunteed to stay the same between reboots
            // Prefer sys path for non-debugging purposes
            std::string deviceNode;
            // To find the sys path:
            // 1) Disconnect all cameras
            // 2) Confirm there are no /dev/video* devices
            // 2) Connect the camera you want to use
            // 3) Run "ls /dev/video*" to verify the device is connected
            // 4) Run "udevadm info -q path -n /dev/video0" to get the sys path
            std::string devicePath;
            int imageWidth, imageHeight, imageFramerate, bitrate;

            std::vector<ParameterWrapper> parameters = {
                    {std::format("{}.address", cameraName), mAddress, "0.0.0.0"},
                    {std::format("{}.port", cameraName), port, 0},
                    {std::format("{}.capture_format", cameraName), captureFormat, "MJPG"},
                    {std::format("{}.codec", cameraName), codec, "H265"},
                    {std::format("{}.dev_node", cameraName), deviceNode, ""},
                    {std::format("{}.dev_path", cameraName), devicePath, ""},
                    {std::format("{}.width", cameraName), imageWidth, 0},
                    {std::format("{}.height", cameraName), imageHeight, 0},
                    {std::format("{}.framerate", cameraName), imageFramerate, 0},
                    {std::format("{}.bitrate", cameraName), bitrate, 0},
                    {std::format("{}.crop_left", cameraName), mCropLeft, 0},
                    {std::format("{}.crop_right", cameraName), mCropRight, 0},
                    {std::format("{}.crop_top", cameraName), mCropTop, 0},
                    {std::format("{}.crop_bottom", cameraName), mCropBottom, 0},

            };

            ParameterWrapper::declareParameters(this, parameters);

            mPort = static_cast<std::uint16_t>(port);
            mCaptureFormat = gst::video::v4l2::getFormatFromStringView(captureFormat);
            mStreamCodec = gst::video::getCodecFromStringView(codec);
            mImageWidth = static_cast<std::uint16_t>(imageWidth);
            mImageHeight = static_cast<std::uint16_t>(imageHeight);
            mImageFramerate = static_cast<std::uint16_t>(imageFramerate);
            mBitrate = static_cast<std::uint64_t>(bitrate);
            mCropEnabled = mCropLeft != 0 || mCropRight != 0 || mCropTop != 0 || mCropBottom != 0;

            // declare_parameter("disable_auto_white_balance", rclcpp::ParameterType::PARAMETER_BOOL);

            if (!devicePath.empty()) {
                deviceNode = findDeviceNode(devicePath);
            }

            gst_init(nullptr, nullptr);

            createLaunchString(deviceNode);
            initPipeline();
            playPipeline();

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
