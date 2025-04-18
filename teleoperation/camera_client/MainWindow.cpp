#include "MainWindow.hpp"

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent) {
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


/*
struct CameraInfo {
    std::string name;
    std::string pipeline;
};
static std::vector<CameraInfo> const CAMERA_INFOS{
        {"Camera 1", "videotestsrc ! video/x-raw,width=1280,height=720,framerate=30/1"},
        {"Camera 2", "videotestsrc ! video/x-raw,width=1280,height=720,framerate=30/1"},
        {"Camera 3", "videotestsrc ! video/x-raw,width=1280,height=720,framerate=30/1"},
        {"Camera 4", "videotestsrc ! video/x-raw,width=1280,height=720,framerate=30/1"},
        {"Camera 5", "videotestsrc ! video/x-raw,width=1280,height=720,framerate=30/1"},
};
for (auto const& cameraInfo: CAMERA_INFOS) {
    mCameraGridWidget->addGstVideoWidget(cameraInfo.name, cameraInfo.pipeline);
    mCameraSelectorWidget->addSelector(cameraInfo.name, mCameraGridWidget->getGstVideoWidget(cameraInfo.name));
}
*/
