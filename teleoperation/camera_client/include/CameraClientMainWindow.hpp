#pragma once

#include "GstRtpVideoCreatorWidget.hpp"
#include "GstVideoWidgets.hpp"
#include "ImagePreview.hpp"
#include "VideoSelectorWidget.hpp"

namespace mrover {
    class CameraConfigWidget;

    class CameraClientMainWindow : public QMainWindow {
        Q_OBJECT

        QScrollArea* mCentralScrollArea;
        GstVideoGridWidget* mCameraGridWidget;

        QDockWidget* mCameraSelectorDock;
        VideoSelectorWidget* mCameraSelectorWidget;

        QDockWidget* mGstRtpVideoCreatorDock;
        GstRtpVideoCreatorWidget* mGstRtpVideoCreatorWidget;

<<<<<<< HEAD
        QDockWidget* mColorPickerDock = nullptr;
        QLabel* mColorCameraLabel = nullptr;
        QFrame* mColorSwatchFrame = nullptr;
        QLabel* mColorRLabel = nullptr;
        QLabel* mColorGLabel = nullptr;
        QLabel* mColorBLabel = nullptr;
        QLabel* mColorHexLabel = nullptr;

        void buildColorPickerDock();
        void onColorPicked(QString const& cameraName, QColor const& color);
=======
        QDockWidget* mCameraConfigDock;
        CameraConfigWidget* mCameraConfigWidget;

        std::unordered_map<std::string, std::vector<std::string>> mConfigs;
>>>>>>> origin/main

    public:
        explicit CameraClientMainWindow(QWidget* parent = nullptr);

        auto createCamera(std::string const& name, std::string const& pipeline, CameraCallbacks callbacks) -> bool;
        auto getCameraGridWidget() -> GstVideoGridWidget*;

        auto setConfigs(std::unordered_map<std::string, std::vector<std::string>>&& configs) -> void;

    public slots:
        void showImagePreview(QString const& cameraName, QImage const& image);
        void loadCameraConfigSlot(std::string const& config);

    signals:
        void closed();

    protected:
        auto closeEvent(QCloseEvent* event) -> void override;
    };

} // namespace mrover
