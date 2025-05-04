#include <rclcpp/rclcpp.hpp>

#include "pch.hpp"

#include "CameraClientMainWindow.hpp"
#include "ImagePreview.hpp"

namespace mrover {
    class CameraClientNode : public rclcpp::Node {

        std::shared_ptr<CameraClientMainWindow> mQtGui;
        std::unordered_map<std::string, rclcpp::Client<srv::MediaControl>::SharedPtr> mMediaControlClients;
        std::unordered_map<std::string, rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr> mImageCaptureClients;
        std::unordered_map<std::string, rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> mImageSubscribers;

        auto imageCallback(std::string const& cameraName, sensor_msgs::msg::Image::ConstSharedPtr const& msg) {
            RCLCPP_INFO(get_logger(), "Received image from camera");
            try {
                auto cvImg = cv_bridge::toCvShare(msg, "bgr8")->image;
                QImage qImg(cvImg.data, cvImg.cols, cvImg.rows, static_cast<int>(cvImg.step), QImage::Format_BGR888);

                auto* imagePreview = new ImagePreview();
                imagePreview->updateImage(qImg);
                imagePreview->setWindowTitle(QString::fromStdString(cameraName) + " - Screenshot");
                imagePreview->setAttribute(Qt::WA_DeleteOnClose);
                imagePreview->show();

            } catch (cv_bridge::Exception const& e) {
                RCLCPP_ERROR(this->get_logger(), "cv_bridge error: %s", e.what());
            }
        }

    public:
        explicit CameraClientNode(std::shared_ptr<CameraClientMainWindow> qtGui) : Node("camera_client"), mQtGui(std::move(qtGui)) {
            RCLCPP_INFO(get_logger(), "Camera client initialized");

            declare_parameter("cameras", rclcpp::ParameterType::PARAMETER_STRING_ARRAY);
            auto cameraNames = get_parameter("cameras").as_string_array();

            declare_parameter("rtp_jitter_ms", 100);
            auto rtpJitterMs = std::chrono::milliseconds(get_parameter("rtp_jitter_ms").as_int());

            for (auto const& cameraName: cameraNames) {
                RCLCPP_INFO(get_logger(), "cameraName: %s", cameraName.c_str());

                if (mMediaControlClients.contains(cameraName)) {
                    RCLCPP_WARN(get_logger(), "Camera %s already exists, skipping", cameraName.c_str());
                    continue;
                }

                declare_parameter(std::format("{}.port", cameraName), rclcpp::ParameterType::PARAMETER_INTEGER);
                std::uint16_t const port = static_cast<std::uint16_t>(this->get_parameter(std::format("{}.port", cameraName)).as_int());

                declare_parameter(std::format("{}.stream.codec", cameraName), rclcpp::ParameterType::PARAMETER_STRING);
                std::string const codec = this->get_parameter(std::format("{}.stream.codec", cameraName)).as_string();

                std::string const pipeline = gst::video::createRtpToRawSrc(port, gst::video::getCodecFromStringView(codec), rtpJitterMs);

                mMediaControlClients.emplace(cameraName, create_client<srv::MediaControl>(std::format("{}_media_control", cameraName)));
                mImageCaptureClients.emplace(cameraName, create_client<std_srvs::srv::Trigger>(std::format("{}_image_capture", cameraName)));
                mImageSubscribers.emplace(cameraName, create_subscription<sensor_msgs::msg::Image>(std::format("{}_image", cameraName), 10, [this, cameraName](sensor_msgs::msg::Image::ConstSharedPtr const& msg) {
                                              imageCallback(cameraName, msg);
                                          }));

                RequestCallback pipelinePauseRequest = [this, cameraName]() {
                    qDebug() << "Pause request for camera" << cameraName.c_str();
                    auto client = mMediaControlClients.find(cameraName);
                    if (client == mMediaControlClients.end()) {
                        RCLCPP_ERROR(get_logger(), "Camera %s not found", cameraName.c_str());
                        return false;
                    }
                    auto request = std::make_shared<srv::MediaControl::Request>();
                    request->command = srv::MediaControl::Request::PAUSE;
                    auto result = client->second->async_send_request(request);

                    // TODO:(owen) check result success
                    return true;
                };

                RequestCallback pipelinePlayRequest = [this, cameraName]() {
                    qDebug() << "Play request for camera" << cameraName.c_str();
                    auto client = mMediaControlClients.find(cameraName);
                    if (client == mMediaControlClients.end()) {
                        RCLCPP_ERROR(get_logger(), "Camera %s not found", cameraName.c_str());
                        return false;
                    }
                    auto request = std::make_shared<srv::MediaControl::Request>();
                    request->command = srv::MediaControl::Request::PLAY;
                    auto result = client->second->async_send_request(request);

                    // TODO:(owen) check result success
                    return true;
                };

                RequestCallback pipelineStopRequest = [this, cameraName]() {
                    qDebug() << "Stop request for camera" << cameraName.c_str();
                    auto client = mMediaControlClients.find(cameraName);
                    if (client == mMediaControlClients.end()) {
                        RCLCPP_ERROR(get_logger(), "Camera %s not found", cameraName.c_str());
                        return false;
                    }
                    auto request = std::make_shared<srv::MediaControl::Request>();
                    request->command = srv::MediaControl::Request::STOP;
                    auto result = client->second->async_send_request(request);

                    // TODO:(owen) check result success
                    return true;
                };

                RequestCallback screenshotRequest = [this, cameraName]() {
                    qDebug() << "Screenshot request for camera" << cameraName.c_str();
                    auto client = mImageCaptureClients.find(cameraName);
                    if (client == mImageCaptureClients.end()) {
                        RCLCPP_ERROR(get_logger(), "Camera %s not found", cameraName.c_str());
                        return false;
                    }
                    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
                    auto result = client->second->async_send_request(request);

                    // TODO:(owen) check result success
                    return true;
                };

                mQtGui->createCamera(cameraName, pipeline);
                mQtGui->getCameraSelectorWidget()->addMediaControls(cameraName, std::move(pipelinePauseRequest), std::move(pipelinePlayRequest), std::move(pipelineStopRequest));
                mQtGui->getCameraSelectorWidget()->addScreenshotButton(cameraName, std::move(screenshotRequest));
            }
        }
    };
} // namespace mrover

auto main(int argc, char** argv) -> int {
    QApplication app(argc, argv);
    auto qtGui = std::make_shared<mrover::CameraClientMainWindow>();
    qtGui->setWindowTitle("MRover Cameras");
    qtGui->setMinimumSize(1280, 720);
    qtGui->show();

    rclcpp::init(argc, argv);
    auto node = std::make_shared<mrover::CameraClientNode>(qtGui);

    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);

    QObject::connect(qtGui.get(), &mrover::CameraClientMainWindow::closed, [&]() {
        rclcpp::shutdown();
    });

    while (rclcpp::ok()) {
        exec.spin_some();
        app.processEvents();
    }

    exec.remove_node(node);

    return EXIT_SUCCESS;
}
