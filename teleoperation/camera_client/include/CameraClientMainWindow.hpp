#pragma once

#include "GstRtpVideoCreatorWidget.hpp"
#include "GstVideoWidgets.hpp"
#include "ImagePreview.hpp"
#include "VideoSelectorWidget.hpp"

namespace mrover {

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

        auto createCamera(std::string const& name, std::string const& pipeline, CameraCallbacks callbacks) -> bool;
        auto getCameraGridWidget() -> GstVideoGridWidget*;

    public slots:
        void showImagePreview(QString const& cameraName, QImage const& image);

    signals:
        void closed();

    protected:
        auto closeEvent(QCloseEvent* event) -> void override;
    };

} // namespace mrover
