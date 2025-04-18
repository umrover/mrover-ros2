#pragma once

#include "GstRtpVideoCreatorWidget.hpp"
#include "GstVideoWidgets.hpp"
#include "VideoSelectorWidget.hpp"

class CameraClientMainWindow : public QMainWindow {
    Q_OBJECT

    QScrollArea* mCentralScrollArea;
    GstVideoGridWidget* mCameraGridWidget;

    QDockWidget* mCameraSelectorDock;
    VideoSelectorWidget* mCameraSelectorWidget;

    QDockWidget* mGstRtpVideoCreatorDock;
    GstRtpVideoCreatorWidget* mGstRtpVideoCreatorWidget;

public:
    explicit CameraClientMainWindow(QWidget* parent = nullptr);

signals:
    void closed();

protected:
    void closeEvent(QCloseEvent* event) override;
};
