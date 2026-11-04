#include "pch.hpp"

#include "CameraClientMainWindow.hpp"
#include "CameraClientNode.hpp"
#include "CameraConfigWidget.hpp"

#include <QDialog>
#include <QDialogButtonBox>
#include <QDoubleSpinBox>

auto main(int argc, char** argv) -> int {
    QApplication app(argc, argv);

    rclcpp::init(argc, argv);

    auto node = std::make_shared<mrover::CameraClientNode>();

    auto mainWindow = std::make_shared<mrover::CameraClientMainWindow>();
    auto secondWindow = std::make_shared<mrover::CameraClientMainWindow>();

    mainWindow->setWindowTitle("MRover Cameras");
    mainWindow->setMinimumSize(1280, 720);

    auto* siteToolbar = mainWindow->addToolBar("Site");
    auto* siteToolbarSide = secondWindow->addToolBar("Site");
    auto* attributeSiteAction = siteToolbar->addAction("Attribute site");
    auto* attributeSiteActionSide = siteToolbarSide->addAction("Attribute site");


    secondWindow->setWindowTitle("Second MRover Cameras");
    secondWindow->setMinimumSize(1280, 720);

    auto openSiteDialog = [](QWidget* parent) {
    QDialog dialog(parent);
    dialog.setWindowTitle("Attribute Site");

    auto* site = new QLineEdit(&dialog);
    site->setText("<no site>");

    auto* latitude = new QDoubleSpinBox(&dialog);
    latitude->setRange(-90.0, 90.0);
    latitude->setDecimals(7);

    auto* longitude = new QDoubleSpinBox(&dialog);
    longitude->setRange(-180.0, 180.0);
    longitude->setDecimals(7);

    auto* form = new QFormLayout;
    form->addRow("Site", site);
    form->addRow("Latitude", latitude);
    form->addRow("Longitude", longitude);

    auto* buttons = new QDialogButtonBox(
        QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dialog);
    QObject::connect(buttons, &QDialogButtonBox::accepted,
                     &dialog, &QDialog::accept);
    QObject::connect(buttons, &QDialogButtonBox::rejected,
                     &dialog, &QDialog::reject);

    auto* layout = new QVBoxLayout(&dialog);
    layout->addLayout(form);
    layout->addWidget(buttons);

    if (dialog.exec() == QDialog::Accepted) {
        qDebug() << "Site:" << site->text()
                 << "Lat:" << latitude->value()
                 << "Lon:" << longitude->value();
    }
};

QObject::connect(attributeSiteAction, &QAction::triggered,
                 mainWindow.get(),
                 [openSiteDialog, parent = mainWindow.get()] {
                     openSiteDialog(parent);
                 });

QObject::connect(attributeSiteActionSide, &QAction::triggered,
                 secondWindow.get(),
                 [openSiteDialog, parent = secondWindow.get()] {
                     openSiteDialog(parent);
                 });

    

    QObject::connect(node.get(), &mrover::CameraClientNode::cameraDiscovered,
                     mainWindow.get(), [mainWindow, secondWindow, node](mrover::CameraInfo const& info) {
                         std::string const& name = info.name;

                         mrover::CameraCallbacks callbacks{
                                 .onHide = [mainWindow, secondWindow, name]() {
                                     auto fun = [name](std::shared_ptr<mrover::CameraClientMainWindow> const& window) {
                                         window->getCameraGridWidget()->hideVideo(name);
                                         if (auto* widget = window->getCameraGridWidget()->getGstVideoWidget(name)) {
                                             widget->stop();
                                         }
                                     };
                                     
                                     if(name.ends_with("_main")){
                                         fun(mainWindow);
                                     }else if(name.ends_with("_second")){
                                         fun(secondWindow);
                                     }
                                     return true; },
                                 .onShow = [mainWindow, secondWindow, name]() {
                                     auto fun = [name](std::shared_ptr<mrover::CameraClientMainWindow> const& window) {
                                         window->getCameraGridWidget()->showVideo(name);
                                         if (auto* widget = window->getCameraGridWidget()->getGstVideoWidget(name)) {
                                             widget->play();
                                         }
                                     };
                                     
                                     if(name.ends_with("_main")){
                                         fun(mainWindow);
                                     }else if(name.ends_with("_second")){
                                         fun(secondWindow);
                                     }
                                     return true; },
                                 .onPause = [node, name]() { return node->requestPause(name); },
                                 .onPlay = [node, name]() { return node->requestPlay(name); },
                                 .onStop = [node, name]() { return node->requestStop(name); },
                                 .onScreenshot = [node, name]() { return node->requestScreenshot(name); },
                                 .onResize = [mainWindow, secondWindow, name](int w, int h) {
                                     auto fun = [name, w, h](std::shared_ptr<mrover::CameraClientMainWindow> const& window) {
                                         window->getCameraGridWidget()->resizeCamera(name, w, h); 
                                     };

                                     if(name.ends_with("_main")){
                                         fun(mainWindow);
                                     }else if(name.ends_with("_second")){
                                         fun(secondWindow);
                                     } },
                                 .onRotate = [mainWindow, secondWindow, name]() {
                                     auto fun = [name](std::shared_ptr<mrover::CameraClientMainWindow> const& window) {
                                         window->getCameraGridWidget()->rotateCamera(name); 
                                     };

                                     if(name.ends_with("_main")){
                                         fun(mainWindow);
                                     }else if(name.ends_with("_second")){
                                         fun(secondWindow);
                                     } }};

                         if (name.ends_with("_main")) {
                             mainWindow->createCamera(name, info.pipeline, std::move(callbacks));
                         } else if (name.ends_with("_second")) {
                             secondWindow->createCamera(name, info.pipeline, std::move(callbacks));
                         }
                     });

    QObject::connect(node.get(), &mrover::CameraClientNode::imageCaptured,
                     mainWindow.get(), &mrover::CameraClientMainWindow::showImagePreview);
    // not sure how to integrate this into the secondary window

    auto configs = node->loadCameraConfigs();

    mainWindow->setConfigs(std::move(configs));
    secondWindow->setConfigs(std::move(configs));

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
    secondWindow->show();

    int const result = QApplication::exec();

    exec.remove_node(node);

    return result;
}
