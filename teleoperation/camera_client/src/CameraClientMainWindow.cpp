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

    connect(mGstRtpVideoCreatorWidget, &GstRtpVideoCreatorWidget::createRequested,
            this, [this](std::string const& name, std::string const& pipeline) {
                qDebug() << "Creating camera with name:" << QString::fromStdString(name) << "and pipeline:" << QString::fromStdString(pipeline);

                if (bool success = createCamera(name, pipeline); !success) {
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

auto CameraClientMainWindow::createCamera(std::string const& name, std::string const& pipeline) -> bool {
    mCameraGridWidget->addGstVideoWidget(name, pipeline);
    if (mCameraGridWidget->isError()) {
        return false;
    }
    mCameraSelectorWidget->addSelector(name);
    connect(mCameraSelectorWidget, &VideoSelectorWidget::selectionChanged,
            this, [this, name](std::string const& selectedName, bool isChecked) {
                if (isChecked) {
                    mCameraGridWidget->showVideo(selectedName);
                    mCameraGridWidget->getGstVideoWidget(selectedName)->play();
                } else {
                    mCameraGridWidget->hideVideo(selectedName);
                    mCameraGridWidget->getGstVideoWidget(selectedName)->stop();
                }
            });
    return true;
}

void CameraClientMainWindow::closeEvent(QCloseEvent* event) {
    emit closed();
    QMainWindow::closeEvent(event);
}
