#include "usb_camera.hpp"

namespace mrover {

    template<typename T>
    auto gstCheck(T* t) -> T* {
        if (!t) throw std::runtime_error{"Failed to create"};
        return t;
    }

    auto gstBusMessage(GstBus*, GstMessage* message, gpointer data) -> gboolean;

    UsbCamera::UsbCamera() : Node{"usb_camera", rclcpp::NodeOptions{}.use_intra_process_comms(true)} {
        try {
            /* Parameters */
            int framerate{};
            std::string device{};
            std::string imageTopicName{};
            std::string cameraInfoTopicName{};
            double watchdogTimeout{};
            bool decodeJpegFromDevice{};

            std::vector<ParameterWrapper> params{
                    {"width", mWidth, 640},
                    {"height", mHeight, 480},
                    {"framerate", framerate, 30},
                    {"device", device, "/dev/long_range_cam"},
                    {"image_topic", imageTopicName, "/usb_camera/image"},
                    {"camera_info_topic", cameraInfoTopicName, "/usb_camera/camera_info"},
                    {"watchdog_timeout", watchdogTimeout, 1.0},
                    {"decode_jpeg_from_device", decodeJpegFromDevice, false}};

            ParameterWrapper::declareParameters(this, params);

            /* Interfaces */
            mImgPub = create_publisher<sensor_msgs::msg::Image>(imageTopicName, 1);
            mCamInfoPub = create_publisher<sensor_msgs::msg::CameraInfo>(cameraInfoTopicName, 1);

            /* Pipeline */
            gst_init(nullptr, nullptr);

            mMainLoop = gstCheck(g_main_loop_new(nullptr, FALSE));

            std::string launch = std::format("v4l2src device={} ", device);
            if (decodeJpegFromDevice) {
                launch += std::format("! image/jpeg,width={},height={},framerate={}/1 ", mWidth, mHeight, framerate);
                if (gst_element_factory_find("nvv4l2decoder")) {
                    launch += "! nvv4l2decoder mjpeg=1 ! nvvidconv ";
                } else {
                    launch += "! jpegdec ! videoconvert ";
                }
                launch += "! video/x-raw,format=YUY2 ";
            } else {
                launch += std::format("! video/x-raw,format=YUY2,width={},height={},framerate={}/1 ", mWidth, mHeight, framerate);
            }
            launch += "! appsink name=streamSink sync=false";
            RCLCPP_INFO_STREAM(get_logger(), std::format("GStreamer launch string: {}", launch));

            mPipeline = gstCheck(gst_parse_launch(launch.c_str(), nullptr));

            GstBus* bus = gstCheck(gst_element_get_bus(mPipeline));
            gst_bus_add_watch(bus, gstBusMessage, this);
            gst_object_unref(bus);

            mStreamSink = gstCheck(gst_bin_get_by_name(GST_BIN(mPipeline), "streamSink"));

            if (gst_element_set_state(mPipeline, GST_STATE_PAUSED) == GST_STATE_CHANGE_FAILURE)
                throw std::runtime_error{"Failed initial pause on GStreamer pipeline"};

            mMainLoopThread = std::thread{[this] {
                RCLCPP_INFO_STREAM(get_logger(), "Entering GStreamer main loop");
                g_main_loop_run(mMainLoop);
                RCLCPP_INFO_STREAM(get_logger(), "Leaving GStreamer main loop");
            }};

            mStreamSinkThread = std::thread{[this] {
                RCLCPP_INFO_STREAM(get_logger(), "Entering stream sink thread");
                pullSampleLoop();
                RCLCPP_INFO_STREAM(get_logger(), "Leaving stream sink thread");
            }};

            if (gst_element_set_state(mPipeline, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE)
                throw std::runtime_error{"Failed to play GStreamer pipeline"};

            RCLCPP_INFO_STREAM(get_logger(), "Initialized and started GStreamer pipeline");

        } catch (std::exception const& e) {
            RCLCPP_ERROR_STREAM(get_logger(), std::format("Exception initializing: {}", e.what()));
            rclcpp::shutdown();
        }
    }

    auto UsbCamera::pullSampleLoop() -> void {
        while (GstSample* sample = gst_app_sink_pull_sample(GST_APP_SINK(mStreamSink))) {
            GstBuffer* buffer = gst_sample_get_buffer(sample);
            GstMapInfo map;
            gst_buffer_map(buffer, &map, GST_MAP_READ);

            sensor_msgs::msg::Image image;
            image.header.stamp = get_clock()->now();
            image.encoding = sensor_msgs::image_encodings::BGRA8;
            image.is_bigendian = __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__;
            image.height = mHeight;
            image.width = mWidth;
            image.step = mWidth * 4;
            image.data.resize(image.step * mHeight);
            // These do not clone the data (due to passing the pointer), just provide a way for OpenCV to access it
            // We are converting directly from GStreamer YUV2 memory to the ROS image BGRA memory
            cv::Mat yuvView{mHeight, mWidth, CV_8UC2, map.data};
            cv::Mat bgraView{mHeight, mWidth, CV_8UC4, image.data.data()};
            cv::cvtColor(yuvView, bgraView, cv::COLOR_YUV2BGRA_YUY2);
            mImgPub->publish(image);

            gst_buffer_unmap(buffer, &map);
            gst_sample_unref(sample);
        }
    }

    UsbCamera::~UsbCamera() {
        if (mMainLoop) {
            g_main_loop_quit(mMainLoop);
            mMainLoopThread.join();
            g_main_loop_unref(mMainLoop);
        }

        if (mPipeline) {
            gst_element_set_state(mPipeline, GST_STATE_NULL);
            mStreamSinkThread.join();
            gst_object_unref(mPipeline);
        }
    }

    auto gstBusMessage(GstBus*, GstMessage* message, gpointer data) -> gboolean {
        auto node = static_cast<UsbCamera*>(data);
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

auto main(int argc, char** argv) -> int {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<mrover::UsbCamera>());
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
