#include "VideoSelectorWidget.hpp"

using namespace mrover;

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

auto VideoSelectorWidget::addCamera(std::string const& name) -> void {
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

    mSelectors.emplace(name, selector);

    // Maintain bottom stretch to keep checkboxes top-aligned
    mSelectorsLayout->takeAt(mSelectorsLayout->count() - 1);
    mSelectorsLayout->addWidget(selector.widget);
    mSelectorsLayout->addStretch();
}

auto VideoSelectorWidget::addVisibilitySelector(std::string const& cameraName, RequestCallback hideRequest, RequestCallback showRequest) -> void {
    if (!mSelectors.contains(cameraName)) {
        qDebug() << "Selector with name" << QString::fromStdString(cameraName) << "does not exist.";
        return;
    }
    Selector& selector = mSelectors.at(cameraName);

    if (mUsingIcons) {
        selector.visibilityCheckBox = new CallbackCheckBox(QIcon::fromTheme("view-reveal-symbolic"), QIcon::fromTheme("view-conceal-symbolic"), selector.widget);
    } else {
        selector.visibilityCheckBox = new CallbackCheckBox("Show", "Hide", selector.widget);
    }

    selector.visibilityCheckBox->setOnCheckCallback(std::move(showRequest));
    selector.visibilityCheckBox->setOnUncheckCallback(std::move(hideRequest));
    selector.visibilityCheckBox->setChecked(true);

    selector.layout->insertWidget(2, selector.visibilityCheckBox);
}

auto VideoSelectorWidget::addMediaControls(std::string const& cameraName, RequestCallback pauseRequest, RequestCallback playRequest, RequestCallback stopRequest) -> void {
    if (!mSelectors.contains(cameraName)) {
        qDebug() << "Selector with name" << QString::fromStdString(cameraName) << "does not exist.";
        return;
    }
    Selector& selector = mSelectors.at(cameraName);

    if (mUsingIcons) {
        selector.playPauseCheckBox = new CallbackCheckBox(QIcon::fromTheme("media-playback-start-symbolic"), QIcon::fromTheme("media-playback-pause-symbolic"), selector.widget);
        selector.stopButton = new QPushButton(QIcon{QIcon::fromTheme("stop-symbolic")}, QString(), selector.widget);
    } else {
        selector.playPauseCheckBox = new CallbackCheckBox("Play", "Pause", selector.widget);
        selector.stopButton = new QPushButton("Stop", selector.widget);
    }
    selector.playPauseCheckBox->setOnCheckCallback(std::move(playRequest));
    selector.playPauseCheckBox->setOnUncheckCallback(std::move(pauseRequest));
    selector.playPauseCheckBox->setChecked(true);

    connect(selector.stopButton, &QPushButton::clicked, this, [this, cameraName, stopRequest]() {
        if (this->mSelectors.contains(cameraName)) {
            mSelectors.at(cameraName).playPauseCheckBox->setChecked(false);
            stopRequest();
        }
    });

    selector.layout->addWidget(selector.playPauseCheckBox);
    selector.layout->addWidget(selector.stopButton);
}

auto VideoSelectorWidget::addScreenshotButton(std::string const& cameraName, RequestCallback screenshotRequest) -> void {
    if (!mSelectors.contains(cameraName)) {
        qDebug() << "Selector with name" << QString::fromStdString(cameraName) << "does not exist.";
        return;
    }
    Selector& selector = mSelectors.at(cameraName);

    if (mUsingIcons) {
        selector.screenshotButton = new QPushButton(QIcon{QIcon::fromTheme("camera-photo-symbolic")}, QString(), selector.widget);
    } else {
        selector.screenshotButton = new QPushButton("Screenshot", selector.widget);
    }

    connect(selector.screenshotButton, &QPushButton::clicked, this, [this, cameraName, screenshotRequest]() {
        if (this->mSelectors.contains(cameraName)) {
            screenshotRequest();
        }
    });

    selector.layout->addWidget(selector.screenshotButton);
}
