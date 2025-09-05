#pragma once

#include "pch.hpp"

#include "CallbackCheckBox.hpp"

namespace mrover {
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

    public:
        explicit VideoSelectorWidget(QWidget* parent = nullptr);

        auto addCamera(std::string const& name) -> void;
        auto addVisibilitySelector(std::string const& cameraName, RequestCallback hideRequest, RequestCallback showRequest) -> void;
        auto addMediaControls(std::string const& cameraName, RequestCallback pauseRequest, RequestCallback playRequest, RequestCallback stopRequest) -> void;
        auto addScreenshotButton(std::string const& cameraName, RequestCallback screenshotRequest) -> void;

    signals:
        void stopClicked(std::string const& name);
        void screenshotClicked(std::string const& name);
    };
} // namespace mrover
