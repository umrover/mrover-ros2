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

        QDockWidget* mColorPickerDock = nullptr;
        QLabel* mColorCameraLabel = nullptr;
        QFrame* mColorSwatchFrame = nullptr;
        QLabel* mColorRLabel = nullptr;
        QLabel* mColorGLabel = nullptr;
        QLabel* mColorBLabel = nullptr;
        QLabel* mColorHexLabel = nullptr;

        void buildColorPickerDock();
        void onColorPicked(QString const& cameraName, QColor const& color);

    public:
        explicit CameraClientMainWindow(QWidget* parent = nullptr);

        auto createCamera(std::string const& name, std::string const& pipeline) -> bool;
        auto getCameraSelectorWidget() -> VideoSelectorWidget*;
        static auto showImagePopup(QImage const& image) -> void;

    signals:
        void closed();

    protected:
        void closeEvent(QCloseEvent* event) override;
    };
}; // namespace mrover
