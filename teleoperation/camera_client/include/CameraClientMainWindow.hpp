#pragma once

#include "CallbackCheckBox.hpp"
#include "GstRtpVideoCreatorWidget.hpp"
#include "GstVideoWidgets.hpp"
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

        auto createCamera(std::string const& name, std::string const& pipeline, bool enableAruco) -> bool;
        auto getCameraSelectorWidget() -> VideoSelectorWidget*;
        static auto showImagePopup(QImage const& image) -> void;

    signals:
        void closed();

    protected:
        void closeEvent(QCloseEvent* event) override;
    };
}; // namespace mrover
