#pragma once

#include <QDockWidget>
#include <QMainWindow>
#include <QScrollArea>

#include "GstRtpVideoCreatorWidget.hpp"
#include "GstVideoGridWidget.hpp"
#include "VideoSelectorWidget.hpp"

class MainWindow : public QMainWindow {
    Q_OBJECT

    QScrollArea* mCentralScrollArea;
    GstVideoGridWidget* mCameraGridWidget;

    QDockWidget* mCameraSelectorDock;
    VideoSelectorWidget* mCameraSelectorWidget;

    QDockWidget* mGstRtpVideoCreatorDock;
    GstRtpVideoCreatorWidget* mGstRtpVideoCreatorWidget;

public:
    explicit MainWindow(QWidget* parent = nullptr);
};
