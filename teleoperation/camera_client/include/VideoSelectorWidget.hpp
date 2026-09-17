#pragma once

#include "pch.hpp"

#include "CallbackCheckBox.hpp"

namespace mrover {

    struct CameraCallbacks {
        RequestCallback onHide;
        RequestCallback onShow;
        RequestCallback onPause;
        RequestCallback onPlay;
        RequestCallback onStop;
        RequestCallback onScreenshot;
        std::function<void(int, int)> onResize;
        std::function<void()> onRotate;
    };

    class VideoSelectorWidget : public QWidget {
        Q_OBJECT

        struct Selector {
            QWidget* widget;
            QHBoxLayout* layout;

            QLabel* nameLabel;
            CallbackCheckBox* visibilityCheckBox;
            CallbackCheckBox* playPauseCheckBox;
            QPushButton* stopButton;
            QPushButton* screenshotButton;
            QSlider* scaleSlider;
            QPushButton* rotateButton;
        };

        bool mUsingIcons;
        std::unordered_map<std::string, Selector> mSelectors;
        QVBoxLayout* mSelectorsLayout;

        auto createVisibilityCheckBox(Selector& selector, CameraCallbacks const& callbacks) -> void;
        auto createMediaControls(Selector& selector, std::string const& cameraName, CameraCallbacks const& callbacks) -> void;
        auto createScreenshotButton(Selector& selector, CameraCallbacks const& callbacks) -> void;
        auto createScaleSlider(Selector& selector, CameraCallbacks const& callbacks) -> void;
        auto createRotateButton(Selector& selector, CameraCallbacks const& callbacks) -> void;

    public:
        explicit VideoSelectorWidget(QWidget* parent = nullptr);

        auto addCamera(std::string const& name, CameraCallbacks callbacks) -> void;
    };

} // namespace mrover
