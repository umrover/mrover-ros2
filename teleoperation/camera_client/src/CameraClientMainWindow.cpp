#include "CameraClientMainWindow.hpp"

using namespace mrover;

CameraClientMainWindow::CameraClientMainWindow(QWidget* parent) : QMainWindow(parent) {
    mCameraGridWidget = new GstVideoGridWidget(this);
    mCentralScrollArea = new QScrollArea(this);

    mCameraSelectorDock = new QDockWidget("Camera Selector", this);
    mCameraSelectorWidget = new VideoSelectorWidget();
    mCameraSelectorDock->setWidget(mCameraSelectorWidget);

    mGstRtpVideoCreatorDock = new QDockWidget("Add Camera", this);
    mGstRtpVideoCreatorWidget = new GstRtpVideoCreatorWidget();
    mGstRtpVideoCreatorDock->setWidget(mGstRtpVideoCreatorWidget);

    mCentralScrollArea->setWidget(mCameraGridWidget);
    mCentralScrollArea->setWidgetResizable(true);
    setCentralWidget(mCentralScrollArea);

    addDockWidget(Qt::LeftDockWidgetArea, mCameraSelectorDock);
    splitDockWidget(mCameraSelectorDock, mGstRtpVideoCreatorDock, Qt::Vertical);

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

auto CameraClientMainWindow::createCamera(std::string const& name, std::string const& pipeline) -> bool {
    mCameraGridWidget->addGstVideoWidget(name, pipeline);
    if (mCameraGridWidget->isError()) {
        return false;
    }
    mCameraSelectorWidget->addCamera(name);

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

    return true;
}

auto CameraClientMainWindow::getCameraSelectorWidget() -> VideoSelectorWidget* {
    return mCameraSelectorWidget;
}

auto CameraClientMainWindow::showImagePopup(QImage const& image) -> void {
    auto imagePopup = new QLabel();
    imagePopup->setWindowFlags(Qt::Window | Qt::WindowStaysOnTopHint);
    imagePopup->setWindowTitle("Screenshot");
    imagePopup->setPixmap(QPixmap::fromImage(image));
    imagePopup->setScaledContents(true);
    imagePopup->resize(image.width(), image.height());
    imagePopup->show();
}

void CameraClientMainWindow::closeEvent(QCloseEvent* event) {
    emit closed();
    QMainWindow::closeEvent(event);
}
