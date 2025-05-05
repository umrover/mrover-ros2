#include "gst_camera_server.hpp"

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

    auto GstCameraServer::deviceImageCallback(sensor_msgs::msg::Image::ConstSharedPtr const& msg) -> void {
        try {
            if (msg->encoding != sensor_msgs::image_encodings::BGRA8) throw std::runtime_error{"Unsupported encoding"};

            cv::Size receivedSize{static_cast<int>(msg->width), static_cast<int>(msg->height)};
            cv::Mat bgraFrame{receivedSize, CV_8UC4, const_cast<std::uint8_t*>(msg->data.data()), msg->step};

            if (cv::Size targetSize{mStreamCaptureFormat.width, mStreamCaptureFormat.height};
                receivedSize != targetSize) {
                RCLCPP_WARN_ONCE(get_logger(), "Image size does not match pipeline app source size, will resize");
                resize(bgraFrame, bgraFrame, targetSize);
            }

            // "step" is the number of bytes (NOT pixels) in an image row
            std::size_t size = bgraFrame.step * bgraFrame.rows;
            GstBuffer* buffer = gstCheck(gst_buffer_new_allocate(nullptr, size, nullptr));
            GstMapInfo info;
            gst_buffer_map(buffer, &info, GST_MAP_WRITE);
            std::memcpy(info.data, bgraFrame.data, size);
            gst_buffer_unmap(buffer, &info);

            gst_app_src_push_buffer(GST_APP_SRC(mDeviceImageSource), buffer);
        } catch (std::exception const& e) {
            RCLCPP_ERROR_STREAM(get_logger(), std::format("Exception encoding frame: {}", e.what()));
            rclcpp::shutdown();
        }
    }


    auto GstCameraServer::createStreamPipeline() -> void {
        gst::PipelineBuilder pipeline;

        auto& [pixelFormat, imageWidth, imageHeight, imageFramerate] = mStreamCaptureFormat;

        // Source
        if (mDeviceImageSubscriber) {
            pipeline.pushBack("appsrc name=imageSource is-live=true");
            pipeline.pushBack(std::format("video/x-raw,format={},width={},height={},framerate={}/1", gst::video::toString(gst::video::RawFormat::BGRA), imageWidth, imageHeight, imageFramerate));
        } else if (!mDeviceNode.empty()) {
            pipeline.pushBack(gst::video::v4l2::createSrc(mDeviceNode, mStreamCaptureFormat));
            // gst::video::v4l2::addProperty("extra-controls", "\"c,white_balance_temperature_auto=0,white_balance_temperature=4000\"")
            // if (mDisableAutoWhiteBalance) mStreamLaunch += "extra-controls=\"c,white_balance_temperature_auto=0,white_balance_temperature=4000\" ";
        } else {
            pixelFormat = gst::video::v4l2::PixelFormat::YUYV;
            pipeline.pushBack("videotestsrc");
            pipeline.pushBack(std::format("video/x-raw,format={},width={},height={},framerate={}/1", gst::video::v4l2::toStringGstType(pixelFormat), imageWidth, imageHeight, imageFramerate));
        }

        // Source decoder and H265 encoder
        if (gst_element_factory_find("nvv4l2h265enc")) {
            // Most likely on the Jetson
            if (pixelFormat == gst::video::v4l2::PixelFormat::MJPG) {
                // TODO(quintin): I had to apply this patch: https://forums.developer.nvidia.com/t/macrosilicon-usb/157777/4
                //                nvv4l2camerasrc only supports UYUV by default, but our cameras are YUY2 (YUYV)
                // "nvv4l2camerasrc device={} "
                // "! video/x-raw(memory:NVMM),format=YUY2,width={},height={},framerate={}/1 "

                // Mostly used with USB cameras, MPEG capture uses way less USB bandwidth
                pipeline.pushBack("nvv4l2decoder", gst::addProperty("mjpeg", 1)); // Hardware-accelerated JPEG decoding, output is apparently some unknown proprietary format
                pipeline.pushBack("nvvidconv");                                   // Convert from proprietary format to NV12 so the encoder understands it
                pipeline.pushBack("video/x-raw(memory:NVMM),format=NV12");

            } else {
                if (mCropEnabled) {
                    pipeline.pushBack(std::format("videocrop left={} right={} top={} bottom={}", mCropLeft, mCropRight, mCropTop, mCropBottom));
                }
                pipeline.pushBack("clockoverlay");

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
            if (gst::video::v4l2::isCompressedPixelFormat(pixelFormat)) {
                pipeline.pushBack(gst::video::createDefaultDecoder(toStringGstType(pixelFormat)));
            } else {
                pipeline.pushBack("videoconvert");
            }

            if (mCropEnabled) {
                pipeline.pushBack(std::format("videocrop left={} right={} top={} bottom={}", mCropLeft, mCropRight, mCropTop, mCropBottom));
            }
            pipeline.pushBack("clockoverlay");

            pipeline.pushBack(gst::video::createDefaultEncoder(mCodec));

            if (mCodec == gst::video::Codec::H264) {
                pipeline.addPropsToElement(pipeline.size() - 1,
                                           gst::addProperty("tune", "zerolatency"),
                                           gst::addProperty("bitrate", mBitrate),
                                           gst::addProperty("name", "encoder"));
            }
        }

        if (mCodec == gst::video::Codec::H265) {
            pipeline.pushBack("h265parse");
        } else if (mCodec == gst::video::Codec::H264) {
            pipeline.pushBack("h264parse");
        }

        pipeline.pushBack(gst::video::createRtpSink(mAddress, mPort, mCodec));

        RCLCPP_INFO_STREAM(get_logger(), std::format("GStreamer stream launch string: {}", pipeline.str()));
        mStreamPipelineWrapper = gst::PipelineWrapper(pipeline.str());
    }

    auto GstCameraServer::createImageCapturePipeline() -> void {
        gst::PipelineBuilder pipeline;

        if (mDeviceNode.empty()) {
            mImageCaptureEnabled = false;
            return;
        }

        pipeline.pushBack(gst::video::v4l2::createSrc(mDeviceNode, mImageCaptureFormat));

        if (isCompressedPixelFormat(mImageCaptureFormat.pixelFormat)) {
            pipeline.pushBack(gst::video::createDefaultDecoder(toStringGstType(mImageCaptureFormat.pixelFormat)));
        } else {
            pipeline.pushBack("videoconvert");
        }
        pipeline.pushBack("video/x-raw,format=BGR");
        if (mCropEnabled) {
            pipeline.pushBack("videocrop",
                              gst::addProperty("left", mCropLeft),
                              gst::addProperty("right", mCropRight),
                              gst::addProperty("top", mCropTop),
                              gst::addProperty("bottom", mCropBottom));
        }
        pipeline.pushBack("clockoverlay");
        pipeline.pushBack("appsink");

        RCLCPP_INFO_STREAM(get_logger(), std::format("GStreamer image capture launch string: {}", pipeline.str()));
        mImageCapturePipelineLaunch = pipeline.str();
    }

    auto GstCameraServer::initStreamPipelines() -> void {
        RCLCPP_INFO_STREAM(get_logger(), "Initializing GStreamer stream pipeline...");

        mMainLoop = gstCheck(g_main_loop_new(nullptr, FALSE));

        mStreamPipelineWrapper.init();
        mStreamPipelineWrapper.addBusWatcher(gstBusMessage, this);
        if (mDeviceImageSubscriber) {
            mDeviceImageSource = gstCheck(gst_bin_get_by_name(GST_BIN(mStreamPipelineWrapper.getPipelineElement()), "imageSource"));
        }

        mMainLoopThread = std::thread{[this] {
            RCLCPP_INFO_STREAM(get_logger(), "Entering GStreamer main loop");
            g_main_loop_run(mMainLoop);
            RCLCPP_INFO_STREAM(get_logger(), "Leaving GStreamer main loop");
        }};

        RCLCPP_INFO_STREAM(get_logger(), "Initialized GStreamer stream pipeline");
    }

    auto GstCameraServer::mediaControlServerCallback(srv::MediaControl::Request::ConstSharedPtr const& req, srv::MediaControl::Response::SharedPtr const& res) -> void {
        if (req->command == srv::MediaControl::Request::STOP) {
            RCLCPP_INFO_STREAM(get_logger(), "Stopping GStreamer pipeline");
            try {
                mStreamPipelineWrapper.stop();
            } catch (std::runtime_error const& e) {
                RCLCPP_ERROR_STREAM(get_logger(), std::format("Failed to stop GStreamer pipeline: {}", e.what()));
                res->success = false;
                return;
            }
        } else if (req->command == srv::MediaControl::Request::PAUSE) {
            RCLCPP_INFO_STREAM(get_logger(), "Pausing GStreamer pipeline");
            try {
                mStreamPipelineWrapper.pause();
            } catch (std::runtime_error const& e) {
                RCLCPP_ERROR_STREAM(get_logger(), std::format("Failed to pause GStreamer pipeline: {}", e.what()));
                res->success = false;
                return;
            }
        } else if (req->command == srv::MediaControl::Request::PLAY) {
            RCLCPP_INFO_STREAM(get_logger(), "Playing GStreamer pipeline");
            try {
                mStreamPipelineWrapper.play();
            } catch (std::runtime_error const& e) {
                RCLCPP_ERROR_STREAM(get_logger(), std::format("Failed to play GStreamer pipeline: {}", e.what()));
                res->success = false;
                return;
            }
        } else {
            RCLCPP_ERROR_STREAM(get_logger(), std::format("Unknown command: {}", req->command));
            res->success = false;
            return;
        }
        res->success = true;
    }

    auto GstCameraServer::imageCaptureServerCallback(std_srvs::srv::Trigger::Request::ConstSharedPtr const&, std_srvs::srv::Trigger::Response::SharedPtr const& res) -> void {
        RCLCPP_INFO_STREAM(get_logger(), "Capture image request received");
        if (!mImageCapturePublisher || !mImageCaptureEnabled) {
            res->success = false;
            return;
        }

        mStreamPipelineWrapper.stop();

        {
            cv::VideoCapture cap(mImageCapturePipelineLaunch, cv::CAP_GSTREAMER);
            if (!cap.isOpened()) {
                RCLCPP_ERROR(this->get_logger(), "Failed to open video stream");
                return;
            }

            cv::Mat img;
            if (!cap.read(img)) {
                RCLCPP_ERROR(this->get_logger(), "Failed to capture image");
                return;
            }

            sensor_msgs::msg::Image msg;
            msg.header.stamp = this->get_clock()->now();
            msg.height = img.rows;
            msg.width = img.cols;
            msg.encoding = "bgr8";
            msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(img.step);
            msg.data.assign(img.datastart, img.dataend);

            mImageCapturePublisher->publish(msg);
            RCLCPP_INFO(this->get_logger(), "Published image");
        }

        mStreamPipelineWrapper.play();

        res->success = true;
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

    GstCameraServer::GstCameraServer([[maybe_unused]] rclcpp::NodeOptions const& options) : Node{"gst_camera_server", rclcpp::NodeOptions{}.use_intra_process_comms(true)} {
        try {
            declare_parameter("camera", rclcpp::ParameterType::PARAMETER_STRING);
            std::string const cameraName = get_parameter("camera").as_string();

            int port;

            // To find the sys path:
            // 1) Disconnect all cameras
            // 2) Confirm there are no /dev/video* devices
            // 2) Connect the camera you want to use
            // 3) Run "ls /dev/video*" to verify the device is connected
            // 4) Run "udevadm info -q path -n /dev/video0" to get the sys path
            std::string devicePath;

            std::string imageTopicName;

            std::string streamPixelFormat, codec;
            int streamImageWidth, streamImageHeight, streamImageFramerate, bitrate;

            std::string imageCapturePixelFormat;
            int imageCaptureImageWidth, imageCaptureImageHeight, imageCaptureImageFramerate;

            std::vector<ParameterWrapper> parameters = {
                    {std::format("{}.address", cameraName), mAddress, "0.0.0.0"},
                    {std::format("{}.port", cameraName), port, 0},
                    {std::format("{}.dev_node", cameraName), mDeviceNode, ""},
                    {std::format("{}.dev_path", cameraName), devicePath, ""},
                    {std::format("{}.image_topic", cameraName), imageTopicName, ""},
                    {std::format("{}.crop_left", cameraName), mCropLeft, 0},
                    {std::format("{}.crop_right", cameraName), mCropRight, 0},
                    {std::format("{}.crop_top", cameraName), mCropTop, 0},
                    {std::format("{}.crop_bottom", cameraName), mCropBottom, 0},
                    {std::format("{}.stream.pixel_format", cameraName), streamPixelFormat, "MJPG"},
                    {std::format("{}.stream.width", cameraName), streamImageWidth, 0},
                    {std::format("{}.stream.height", cameraName), streamImageHeight, 0},
                    {std::format("{}.stream.framerate", cameraName), streamImageFramerate, 0},
                    {std::format("{}.stream.codec", cameraName), codec, "H265"},
                    {std::format("{}.stream.bitrate", cameraName), bitrate, 0},
                    {std::format("{}.image_capture.pixel_format", cameraName), imageCapturePixelFormat, "MJPG"},
                    {std::format("{}.image_capture.width", cameraName), imageCaptureImageWidth, 0},
                    {std::format("{}.image_capture.height", cameraName), imageCaptureImageHeight, 0},
                    {std::format("{}.image_capture.framerate", cameraName), imageCaptureImageFramerate, 0},
            };

            ParameterWrapper::declareParameters(this, parameters);

            if (!imageTopicName.empty()) {
                mDeviceImageSubscriber = create_subscription<sensor_msgs::msg::Image>(
                        imageTopicName, 1,
                        [this](sensor_msgs::msg::Image::ConstSharedPtr const& msg) {
                            deviceImageCallback(msg);
                        });
            } else {
                if (!devicePath.empty()) {
                    mDeviceNode = findDeviceNode(devicePath);
                }
            }


            mPort = static_cast<std::uint16_t>(port);
            mStreamCaptureFormat = gst::video::v4l2::CaptureFormat{
                    .pixelFormat = gst::video::v4l2::getPixelFormatFromStringView(streamPixelFormat),
                    .width = static_cast<std::uint16_t>(streamImageWidth),
                    .height = static_cast<std::uint16_t>(streamImageHeight),
                    .framerate = static_cast<std::uint16_t>(streamImageFramerate),
            };
            mCodec = gst::video::getCodecFromStringView(codec);
            mBitrate = static_cast<std::uint64_t>(bitrate);

            if (imageCaptureImageWidth == 0 || imageCaptureImageHeight == 0 || imageCaptureImageFramerate == 0) {
                mImageCaptureEnabled = false;
            } else {
                mImageCaptureEnabled = true;
            }

            mImageCaptureFormat = gst::video::v4l2::CaptureFormat{
                    .pixelFormat = gst::video::v4l2::getPixelFormatFromStringView(imageCapturePixelFormat),
                    .width = static_cast<std::uint16_t>(imageCaptureImageWidth),
                    .height = static_cast<std::uint16_t>(imageCaptureImageHeight),
                    .framerate = static_cast<std::uint16_t>(imageCaptureImageFramerate),
            };

            mCropEnabled = mCropLeft != 0 || mCropRight != 0 || mCropTop != 0 || mCropBottom != 0;
            // declare_parameter("disable_auto_white_balance", rclcpp::ParameterType::PARAMETER_BOOL);

            mMediaControlServer = create_service<srv::MediaControl>(
                    std::format("{}_media_control", cameraName),
                    [this](srv::MediaControl::Request::ConstSharedPtr const& req,
                           srv::MediaControl::Response::SharedPtr const& res) {
                        mediaControlServerCallback(req, res);
                    });

            mImageCaptureServer = create_service<std_srvs::srv::Trigger>(
                    std::format("{}_image_capture", cameraName),
                    [this](std_srvs::srv::Trigger::Request::SharedPtr const req,
                           std_srvs::srv::Trigger::Response::SharedPtr res) {
                        imageCaptureServerCallback(req, res);
                    });
            mImageCapturePublisher = create_publisher<sensor_msgs::msg::Image>(std::format("{}_image", cameraName), 1);

            gst_init(nullptr, nullptr);

            createStreamPipeline();
            createImageCapturePipeline();
            initStreamPipelines();
            mStreamPipelineWrapper.play();

        } catch (std::exception const& e) {
            RCLCPP_ERROR_STREAM(get_logger(), std::format("Exception initializing GStreamer V4L2 streamer: {}", e.what()));
            rclcpp::shutdown();
        }
    }

    GstCameraServer::~GstCameraServer() {
        if (mMainLoop) {
            g_main_loop_quit(mMainLoop);
            mMainLoopThread.join();
            g_main_loop_unref(mMainLoop);
        }
    }

    auto gstBusMessage(GstBus*, GstMessage* message, gpointer data) -> gboolean {
        auto node = static_cast<GstCameraServer*>(data);
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
RCLCPP_COMPONENTS_REGISTER_NODE(mrover::GstCameraServer)
