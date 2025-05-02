#include <rclcpp/rclcpp.hpp>

#include "pch.hpp"

#include "CameraClientMainWindow.hpp"

namespace mrover {
    class CameraClientNode : public rclcpp::Node {

        std::shared_ptr<CameraClientMainWindow> mQtGui;
        std::unordered_map<std::string, rclcpp::Client<srv::MediaControl>::SharedPtr> mMediaControlClients;

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
                declare_parameter(std::format("{}.codec", cameraName), rclcpp::ParameterType::PARAMETER_STRING);

                std::uint16_t const port = static_cast<std::uint16_t>(this->get_parameter(std::format("{}.port", cameraName)).as_int());
                std::string const codec = this->get_parameter(std::format("{}.codec", cameraName)).as_string();

                std::string const pipeline = gst::video::createRtpToRawSrc(port, gst::video::getCodecFromStringView(codec), rtpJitterMs);

                mMediaControlClients.emplace(cameraName, create_client<srv::MediaControl>(std::format("{}_media_control", cameraName)));

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
                    return true;
                };

                mQtGui->createCamera(cameraName, pipeline);
                mQtGui->getCameraSelectorWidget()->addMediaControls(cameraName, std::move(pipelinePauseRequest), std::move(pipelinePlayRequest), std::move(pipelineStopRequest), []() { return true; });
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
