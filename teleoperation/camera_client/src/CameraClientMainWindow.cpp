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

<<<<<<< HEAD
    buildColorPickerDock();

    connect(mGstRtpVideoCreatorWidget, &GstRtpVideoCreatorWidget::createRequested,
            this, [this](std::string const& name, std::string const& pipeline) {
                bool success = createCamera(name, pipeline);
                QMetaObject::invokeMethod(mGstRtpVideoCreatorWidget, "onCreateResult",
                                          Qt::QueuedConnection,
                                          Q_ARG(bool, success),
                                          Q_ARG(QString, success ? QString{} : mCameraGridWidget->errorString()));
            });

    connect(mCameraGridWidget, &GstVideoGridWidget::colorPicked,
            this, &CameraClientMainWindow::onColorPicked);
}

void CameraClientMainWindow::buildColorPickerDock() {
    mColorPickerDock = new QDockWidget("Color Picker", this);

    auto* panel = new QWidget();
    auto* panelLayout = new QVBoxLayout(panel);
    panelLayout->setAlignment(Qt::AlignTop);
    panelLayout->setSpacing(8);

    mColorCameraLabel = new QLabel("Hover over a camera feed");
    mColorCameraLabel->setAlignment(Qt::AlignCenter);
    mColorCameraLabel->setWordWrap(true);

    mColorSwatchFrame = new QFrame();
    mColorSwatchFrame->setFixedSize(80, 80);
    mColorSwatchFrame->setFrameShape(QFrame::Box);
    mColorSwatchFrame->setStyleSheet("background-color: #000000; border: 1px solid gray;");

    auto* swatchRow = new QHBoxLayout();
    swatchRow->addWidget(mColorSwatchFrame, 0, Qt::AlignCenter);

    auto* valuesForm = new QFormLayout();
    mColorRLabel = new QLabel("--");
    mColorGLabel = new QLabel("--");
    mColorBLabel = new QLabel("--");
    mColorHexLabel = new QLabel("--");
    valuesForm->addRow("R:", mColorRLabel);
    valuesForm->addRow("G:", mColorGLabel);
    valuesForm->addRow("B:", mColorBLabel);
    valuesForm->addRow("Hex:", mColorHexLabel);

    panelLayout->addWidget(mColorCameraLabel);
    panelLayout->addLayout(swatchRow);
    panelLayout->addLayout(valuesForm);

    panel->setLayout(panelLayout);
    mColorPickerDock->setWidget(panel);
    addDockWidget(Qt::RightDockWidgetArea, mColorPickerDock);
}

void CameraClientMainWindow::onColorPicked(QString const& cameraName, QColor const& color) {
    mColorCameraLabel->setText(QString("Camera: %1").arg(cameraName));
    mColorSwatchFrame->setStyleSheet(
        QString("background-color: %1; border: 1px solid gray;").arg(color.name()));
    mColorRLabel->setText(QString::number(color.red()));
    mColorGLabel->setText(QString::number(color.green()));
    mColorBLabel->setText(QString::number(color.blue()));
    mColorHexLabel->setText(color.name().toUpper());
}
=======
        addDockWidget(Qt::LeftDockWidgetArea, mCameraSelectorDock);
        splitDockWidget(mCameraSelectorDock, mGstRtpVideoCreatorDock, Qt::Vertical);
        splitDockWidget(mCameraSelectorDock, mCameraConfigDock, Qt::Vertical);

        connect(mGstRtpVideoCreatorWidget, &GstRtpVideoCreatorWidget::createRequested,
                this, [this](std::string const& name, std::string const& pipeline) {
                    qDebug() << "Creating camera with name:" << QString::fromStdString(name) << "and pipeline:" << QString::fromStdString(pipeline);
>>>>>>> origin/main

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

<<<<<<< HEAD
    // clang-format off
    mCameraSelectorWidget->addVisibilitySelector(name,
                                                 [this, name]() {
                                                     mCameraGridWidget->hideVideo(name);
                                                     mCameraGridWidget->getGstVideoWidget(name)->stop();
                                                     return true;
                                                 },
                                                 [this, name]() {
                                                     mCameraGridWidget->showVideo(name);
                                                     mCameraGridWidget->getGstVideoWidget(name)->play();
                                                     return true;
                                                 });
    // clang-format on
=======
    auto CameraClientMainWindow::createCamera(std::string const& name, std::string const& pipeline, CameraCallbacks callbacks) -> bool {
        mCameraGridWidget->addGstVideoWidget(name, pipeline);
        if (mCameraGridWidget->isError()) {
            return false;
        }
        mCameraSelectorWidget->addCamera(name, std::move(callbacks));
        return true;
    }
>>>>>>> origin/main

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
