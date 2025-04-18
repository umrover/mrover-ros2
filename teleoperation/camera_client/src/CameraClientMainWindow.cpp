#include "CameraClientMainWindow.hpp"

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

                QWidget* addedCameraWidget = mCameraGridWidget->addGstVideoWidget(name, pipeline);
                if (mCameraGridWidget->isError()) {
                    QMetaObject::invokeMethod(mGstRtpVideoCreatorWidget, "onCreateResult",
                                              Qt::QueuedConnection,
                                              Q_ARG(bool, false),
                                              Q_ARG(QString, mCameraGridWidget->errorString()));
                    return;
                }

                mCameraSelectorWidget->addSelector(name, addedCameraWidget);

                QMetaObject::invokeMethod(mGstRtpVideoCreatorWidget, "onCreateResult",
                                          Qt::QueuedConnection,
                                          Q_ARG(bool, true));
            });
}

void CameraClientMainWindow::closeEvent(QCloseEvent* event) {
    emit closed();
    QMainWindow::closeEvent(event);
}
