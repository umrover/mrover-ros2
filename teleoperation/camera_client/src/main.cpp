#include <QTimer>

#include <rclcpp/rclcpp.hpp>

#include "pch.hpp"

#include <gst/gst.h>

#include <X11/Xlib.h>
#undef Bool
#undef Status
#undef CursorShape
#undef None
#undef KeyPress
#undef KeyRelease
#undef FocusIn
#undef FocusOut
#undef FontChange
#undef Expose
#undef Unsorted

#include "CameraClientMainWindow.hpp"
#include "CameraClientNode.hpp"

namespace {
    XErrorHandler gPreviousHandler = nullptr;

    int filterXErrors(Display* dpy, XErrorEvent* event) {
        if (event->error_code == BadWindow || event->error_code == BadDrawable) return 0;
        if (gPreviousHandler) return gPreviousHandler(dpy, event);
        return 0;
    }
}

auto main(int argc, char** argv) -> int {
    QApplication app(argc, argv);

    gPreviousHandler = XSetErrorHandler(filterXErrors);
    gst_init(nullptr, nullptr);

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

                         auto* videoWidget = mainWindow->getCameraGridWidget()->getGstVideoWidget(name);
                         if (!videoWidget) return;

                         if (info.imageWidth > 0 && info.imageHeight > 0) {
                             videoWidget->setImageSize(info.imageWidth, info.imageHeight);
                         }

                         if (name == "zed") {
                             auto* panel = mainWindow->getClickIkPanel();
                             QObject::connect(videoWidget, &mrover::GstVideoWidget::clicked,
                                              panel, [panel, nodePtr = node.get()](std::uint32_t x, std::uint32_t y) {
                                                  if (panel->canSendClick()) {
                                                      panel->updateClickPosition(x, y);
                                                      panel->markRunning();
                                                      nodePtr->sendClickIk(x, y);
                                                  }
                                              });
                         }
                     });

    QObject::connect(node.get(), &mrover::CameraClientNode::imageCaptured,
                     mainWindow.get(), &mrover::CameraClientMainWindow::showImagePreview);

    QObject::connect(node.get(), &mrover::CameraClientNode::clickIkFeedback,
                     mainWindow->getClickIkPanel(), &mrover::ClickIkPanel::updateFeedback);
    QObject::connect(node.get(), &mrover::CameraClientNode::clickIkResult,
                     mainWindow->getClickIkPanel(), &mrover::ClickIkPanel::updateResult);

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
        } else {
            QApplication::quit();
        }
    });
    spinTimer.start(10); // 10ms

    mainWindow->show();

    int const result = QApplication::exec();

    exec.remove_node(node);

    return result;
}
