#include <rclcpp/rclcpp.hpp>

#include "pch.hpp"

#include "CameraClientMainWindow.hpp"

namespace mrover {
    class CameraClientNode : public rclcpp::Node {

        std::shared_ptr<CameraClientMainWindow> mQtGui;

    public:
        explicit CameraClientNode(std::shared_ptr<CameraClientMainWindow> qtGui) : Node("camera_client"), mQtGui(std::move(qtGui)) {
            RCLCPP_INFO(get_logger(), "Camera client initialized");

            declare_parameter("cameras", rclcpp::ParameterType::PARAMETER_STRING_ARRAY);

            rclcpp::Parameter cameraNameParams;
            this->get_parameter("cameras", cameraNameParams);

            auto cameraNames = cameraNameParams.as_string_array();
            for (auto const& cameraName: cameraNames) {
                RCLCPP_INFO(get_logger(), "cameraName: %s", cameraName.c_str());
                declare_parameter(std::format("{}.port", cameraName), rclcpp::ParameterType::PARAMETER_INTEGER);
                declare_parameter(std::format("{}.codec", cameraName), rclcpp::ParameterType::PARAMETER_STRING);

                std::uint16_t const port = static_cast<std::uint16_t>(this->get_parameter(std::format("{}.port", cameraName)).as_int());
                std::string const codec = this->get_parameter(std::format("{}.codec", cameraName)).as_string();

                std::string const pipeline = Gst::PipelineStrings::createRtpToRawString(port, Gst::getVideoCodecFromString(codec));

                mQtGui->createCamera(cameraName, pipeline);
            }
        }
    };
} // namespace mrover

auto main(int argc, char** argv) -> int {
    QApplication app(argc, argv);
    auto qtGui = std::make_shared<CameraClientMainWindow>();
    qtGui->setWindowTitle("MRover Cameras");
    qtGui->setMinimumSize(1280, 720);
    qtGui->show();

    rclcpp::init(argc, argv);
    auto node = std::make_shared<mrover::CameraClientNode>(qtGui);

    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);

    QObject::connect(qtGui.get(), &CameraClientMainWindow::closed, [&]() {
        rclcpp::shutdown();
    });

    while (rclcpp::ok()) {
        exec.spin_some();
        app.processEvents();
    }

    exec.remove_node(node);

    return EXIT_SUCCESS;
}
