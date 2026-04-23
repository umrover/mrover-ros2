#include "CameraClientMainWindow.hpp"
#include "CameraClientNode.hpp"
#include "CameraConfigWidget.hpp"

namespace mrover {

    CameraClientMainWindow::CameraClientMainWindow(QWidget* parent) : QMainWindow(parent) {
        mCameraGridWidget = new GstVideoGridWidget(this);
        mCentralScrollArea = new QScrollArea(this);

        mCameraSelectorDock = new QDockWidget("Camera Selector", this);
        mCameraSelectorWidget = new VideoSelectorWidget();
        mCameraSelectorDock->setWidget(mCameraSelectorWidget);

        mGstRtpVideoCreatorDock = new QDockWidget("Add Camera", this);
        mGstRtpVideoCreatorWidget = new GstRtpVideoCreatorWidget();
        mGstRtpVideoCreatorDock->setWidget(mGstRtpVideoCreatorWidget);
        QPalette pal = QPalette();
        pal.setColor(QPalette::Window, QApplication::palette().color(QPalette::Window));
        mGstRtpVideoCreatorDock->setAutoFillBackground(true);
        mGstRtpVideoCreatorDock->setPalette(pal);

        // Dock for selecting configuration
        mCameraConfigDock = new QDockWidget("Config Selector", this);
        mCameraConfigWidget = new CameraConfigWidget();
        mCameraConfigDock->setWidget(mCameraConfigWidget);
        QObject::connect(mCameraConfigWidget, &CameraConfigWidget::loadCameraConfigSignal, this, &CameraClientMainWindow::loadCameraConfigSlot);

        mCentralScrollArea->setWidget(mCameraGridWidget);
        mCentralScrollArea->setWidgetResizable(true);
        setCentralWidget(mCentralScrollArea);

        addDockWidget(Qt::LeftDockWidgetArea, mCameraSelectorDock);
        splitDockWidget(mCameraSelectorDock, mGstRtpVideoCreatorDock, Qt::Vertical);
        splitDockWidget(mCameraSelectorDock, mCameraConfigDock, Qt::Vertical);

        connect(mGstRtpVideoCreatorWidget, &GstRtpVideoCreatorWidget::createRequested,
                this, [this](std::string const& name, std::string const& pipeline) {
                    qDebug() << "Creating camera with name:" << QString::fromStdString(name) << "and pipeline:" << QString::fromStdString(pipeline);

                    CameraCallbacks callbacks{
                            .onHide = [this, name]() {
                                mCameraGridWidget->hideVideo(name);
                                if (auto* widget = mCameraGridWidget->getGstVideoWidget(name)) {
                                    widget->stop();
                                }
                                return true; },
                            .onShow = [this, name]() {
                                mCameraGridWidget->showVideo(name);
                                if (auto* widget = mCameraGridWidget->getGstVideoWidget(name)) {
                                    widget->play();
                                }
                                return true; },
                            .onPause = []() { return true; },
                            .onPlay = []() { return true; },
                            .onStop = []() { return true; },
                            .onScreenshot = []() { return true; },
                    };

                    if (bool success = createCamera(name, pipeline, std::move(callbacks)); !success) {
                        QMetaObject::invokeMethod(mGstRtpVideoCreatorWidget, "onCreateResult",
                                                  Qt::QueuedConnection,
                                                  Q_ARG(bool, false),
                                                  Q_ARG(QString, mCameraGridWidget->errorString()));
                    } else {
                        QMetaObject::invokeMethod(mGstRtpVideoCreatorWidget, "onCreateResult",
                                                  Qt::QueuedConnection,
                                                  Q_ARG(bool, true));
                    }
                });
    }

    auto CameraClientMainWindow::createCamera(std::string const& name, std::string const& pipeline, CameraCallbacks callbacks) -> bool {
        mCameraGridWidget->addGstVideoWidget(name, pipeline);
        if (mCameraGridWidget->isError()) {
            return false;
        }
        mCameraSelectorWidget->addCamera(name, std::move(callbacks));
        return true;
    }

    auto CameraClientMainWindow::getCameraGridWidget() -> GstVideoGridWidget* {
        return mCameraGridWidget;
    }

    void CameraClientMainWindow::showImagePreview(QString const& cameraName, QImage const& image) {
        auto* preview = new ImagePreview();
        preview->updateImage(image);
        preview->setWindowTitle(cameraName + " - Screenshot");
        preview->setAttribute(Qt::WA_DeleteOnClose);
        preview->show();
    }

    auto CameraClientMainWindow::closeEvent(QCloseEvent* event) -> void {
        emit closed();
        QMainWindow::closeEvent(event);
    }

    void CameraClientMainWindow::loadCameraConfigSlot(std::string const& config) {
        mCameraGridWidget->hideAll();
        for (auto const& name: mConfigs[config]) {
            mCameraGridWidget->showVideo(name);
        }
    }

    auto CameraClientMainWindow::setConfigs(std::unordered_map<std::string, std::vector<std::string>>&& configs) -> void {
        mConfigs = configs;
        loadCameraConfigSlot(CAMERA_CONFIGS[CAMERA_CONFIG::ARM]);
    }
} // namespace mrover
