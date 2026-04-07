#include "VideoSelectorWidget.hpp"

namespace mrover {

    VideoSelectorWidget::VideoSelectorWidget(QWidget* parent)
        : QWidget(parent) {
        mSelectorsLayout = new QVBoxLayout(this);
        mSelectorsLayout->setSpacing(10);
        mSelectorsLayout->addStretch();
        setLayout(mSelectorsLayout);

        mUsingIcons = (QIcon::hasThemeIcon("pause-symbolic") &&
                       QIcon::hasThemeIcon("play-symbolic") &&
                       QIcon::hasThemeIcon("stop-symbolic") &&
                       QIcon::hasThemeIcon("camera-photo-symbolic"));
    }

    auto VideoSelectorWidget::addCamera(std::string const& name, CameraCallbacks callbacks) -> void {
        if (mSelectors.contains(name)) {
            qDebug() << "Selector with name" << QString::fromStdString(name) << "already exists.";
            return;
        }

        Selector selector{};
        selector.widget = new QWidget(this);
        selector.layout = new QHBoxLayout(selector.widget);
        selector.widget->setLayout(selector.layout);

        selector.nameLabel = new QLabel(QString::fromStdString(name), selector.widget);
        selector.layout->addWidget(selector.nameLabel);
        selector.layout->addStretch();

        createVisibilityCheckBox(selector, callbacks);
        createMediaControls(selector, name, callbacks);
        createScreenshotButton(selector, callbacks);

        mSelectors.emplace(name, selector);

        // bottom stretch to keep selectors top-aligned
        mSelectorsLayout->takeAt(mSelectorsLayout->count() - 1);
        mSelectorsLayout->addWidget(selector.widget);
        mSelectorsLayout->addStretch();
    }

    auto VideoSelectorWidget::createVisibilityCheckBox(Selector& selector, CameraCallbacks const& callbacks) -> void {
        if (mUsingIcons) {
            selector.visibilityCheckBox = new CallbackCheckBox(
                    QIcon::fromTheme("view-reveal-symbolic"),
                    QIcon::fromTheme("view-conceal-symbolic"),
                    selector.widget);
        } else {
            selector.visibilityCheckBox = new CallbackCheckBox("Show", "Hide", selector.widget);
        }

        selector.visibilityCheckBox->setOnCheckCallback(callbacks.onShow);
        selector.visibilityCheckBox->setOnUncheckCallback(callbacks.onHide);
        selector.visibilityCheckBox->setChecked(true);

        selector.layout->addWidget(selector.visibilityCheckBox);
    }

    auto VideoSelectorWidget::createMediaControls(Selector& selector, std::string const& cameraName, CameraCallbacks const& callbacks) -> void {
        if (mUsingIcons) {
            selector.playPauseCheckBox = new CallbackCheckBox(
                    QIcon::fromTheme("media-playback-start-symbolic"),
                    QIcon::fromTheme("media-playback-pause-symbolic"),
                    selector.widget);
            selector.stopButton = new QPushButton(QIcon::fromTheme("stop-symbolic"), QString(), selector.widget);
        } else {
            selector.playPauseCheckBox = new CallbackCheckBox("Play", "Pause", selector.widget);
            selector.stopButton = new QPushButton("Stop", selector.widget);
        }

        selector.playPauseCheckBox->setOnCheckCallback(callbacks.onPlay);
        selector.playPauseCheckBox->setOnUncheckCallback(callbacks.onPause);
        selector.playPauseCheckBox->setChecked(true);

        auto stopCallback = callbacks.onStop;
        connect(selector.stopButton, &QPushButton::clicked, this, [this, cameraName, stopCallback]() {
            if (mSelectors.contains(cameraName)) {
                mSelectors.at(cameraName).playPauseCheckBox->setChecked(false);
                stopCallback();
            }
        });

        selector.layout->addWidget(selector.playPauseCheckBox);
        selector.layout->addWidget(selector.stopButton);
    }

    auto VideoSelectorWidget::createScreenshotButton(Selector& selector, CameraCallbacks const& callbacks) -> void {
        if (mUsingIcons) {
            selector.screenshotButton = new QPushButton(QIcon::fromTheme("camera-photo-symbolic"), QString(), selector.widget);
        } else {
            selector.screenshotButton = new QPushButton("Screenshot", selector.widget);
        }

        auto screenshotCallback = callbacks.onScreenshot;
        connect(selector.screenshotButton, &QPushButton::clicked, this, [screenshotCallback]() {
            screenshotCallback();
        });

        selector.layout->addWidget(selector.screenshotButton);
    }

} // namespace mrover
