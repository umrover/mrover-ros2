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
        };

        bool mUsingIcons;
        std::unordered_map<std::string, Selector> mSelectors;
        QVBoxLayout* mSelectorsLayout;

        auto createVisibilityCheckBox(Selector& selector, CameraCallbacks const& callbacks) -> void;
        auto createMediaControls(Selector& selector, std::string const& cameraName, CameraCallbacks const& callbacks) -> void;
        auto createScreenshotButton(Selector& selector, CameraCallbacks const& callbacks) -> void;

    public:
        explicit VideoSelectorWidget(QWidget* parent = nullptr);

        auto addCamera(std::string const& name, CameraCallbacks callbacks) -> void;
    };

} // namespace mrover
