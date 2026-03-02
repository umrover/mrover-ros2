#include <QTimer>

#include <rclcpp/rclcpp.hpp>

#include "pch.hpp"

#include "CameraClientMainWindow.hpp"
#include "CameraClientNode.hpp"

auto main(int argc, char** argv) -> int {
    QApplication app(argc, argv);

    rclcpp::init(argc, argv);

    auto mainWindow = std::make_shared<mrover::CameraClientMainWindow>();
    mainWindow->setWindowTitle("MRover Cameras");
    mainWindow->setMinimumSize(1280, 720);

    auto node = std::make_shared<mrover::CameraClientNode>();

    QObject::connect(node.get(), &mrover::CameraClientNode::cameraDiscovered,
                     mainWindow.get(), [mainWindow, node](mrover::CameraInfo const& info) {
                         std::string const& name = info.name;

                         mrover::CameraCallbacks callbacks{
                                 .onHide = [mainWindow, name]() {
                                     mainWindow->getCameraGridWidget()->hideVideo(name);
                                     if (auto* widget = mainWindow->getCameraGridWidget()->getGstVideoWidget(name)) {
                                         widget->stop();
                                     }
                                     return true; },
                                 .onShow = [mainWindow, name]() {
                                     mainWindow->getCameraGridWidget()->showVideo(name);
                                     if (auto* widget = mainWindow->getCameraGridWidget()->getGstVideoWidget(name)) {
                                         widget->play();
                                     }
                                     return true; },
                                 .onPause = [node, name]() { return node->requestPause(name); },
                                 .onPlay = [node, name]() { return node->requestPlay(name); },
                                 .onStop = [node, name]() { return node->requestStop(name); },
                                 .onScreenshot = [node, name]() { return node->requestScreenshot(name); },
                         };

                         mainWindow->createCamera(name, info.pipeline, std::move(callbacks));
                     });

    QObject::connect(node.get(), &mrover::CameraClientNode::imageCaptured,
                     mainWindow.get(), &mrover::CameraClientMainWindow::showImagePreview);

    node->discoverCameras();

    QObject::connect(mainWindow.get(), &mrover::CameraClientMainWindow::closed, []() {
        rclcpp::shutdown();
    });

    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);

    QTimer spinTimer;
    QObject::connect(&spinTimer, &QTimer::timeout, [&exec]() {
        if (rclcpp::ok()) {
            exec.spin_some();
        }
    });
    spinTimer.start(10); // 10ms

    mainWindow->show();

    int const result = QApplication::exec();

    exec.remove_node(node);

    return result;
}
