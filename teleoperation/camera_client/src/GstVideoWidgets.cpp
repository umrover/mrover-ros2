#include "GstVideoWidgets.hpp"

GstVideoWidget::GstVideoWidget(QWidget* parent) : QVideoWidget(parent) {
    mPlayer = new QMediaPlayer(this);
    mPlayer->setVideoOutput(this);
}

auto GstVideoWidget::setGstPipeline(std::string const& pipeline) -> void {
    mPlayer->setMedia(QUrl(std::format("gst-pipeline: {} ! videoconvert ! xvimagesink name=\"qtvideosink\"", pipeline).c_str()));
    play();
}

auto GstVideoWidget::errorString() const -> QString {
    return mPlayer->errorString();
}

auto GstVideoWidget::error() const -> QMediaPlayer::Error {
    return mPlayer->error();
}

auto GstVideoWidget::isError() const -> bool {
    return mPlayer->error() != QMediaPlayer::NoError;
}

auto GstVideoWidget::play() -> void {
    mPlayer->play();
}

auto GstVideoWidget::pause() -> void {
    mPlayer->pause();
}

auto GstVideoWidget::stop() -> void {
    mPlayer->stop();
}

// --------------------------------------------

GstVideoBoxWidget::GstVideoBoxWidget(std::string const& name, QWidget* parent)
    : QWidget(parent) {
    mLayout = new QVBoxLayout(this);
    mLabel = new QLabel(QString::fromStdString(name), this);
    mGstVideoWidget = new GstVideoWidget(this);
    mGstVideoWidget->setMinimumSize(640, 360);

    mLayout->addWidget(mLabel, 0);
    mLayout->addWidget(mGstVideoWidget, 1);

    setLayout(mLayout);
}

auto GstVideoBoxWidget::getGstVideoWidget() -> GstVideoWidget* {
    return mGstVideoWidget;
}

auto GstVideoBoxWidget::getLabel() -> QLabel* {
    return mLabel;
}

auto GstVideoBoxWidget::setTitle(std::string const& title) -> void {
    mLabel->setText(QString::fromStdString(title));
}

// --------------------------------------------

GstVideoGridWidget::GstVideoGridWidget(QWidget* parent)
    : QWidget(parent), mError(NoError) {
    mMainLayout = new QGridLayout(this);
    mMainLayout->setContentsMargins(10, -1, 10, -1);
    mMainLayout->setSpacing(20);
    setLayout(mMainLayout);
}

auto GstVideoGridWidget::addGstVideoWidget(std::string const& name, std::string const& pipeline) -> bool {
    if (mGstVideoBoxes.contains(name)) {
        mError = ExistsError;
        mErrorString = "Camera name already exists";
        return false;
    }

    auto* gstVideoBoxWidget = new GstVideoBoxWidget(name, this);
    GstVideoWidget* gstVideoWidget = gstVideoBoxWidget->getGstVideoWidget();
    gstVideoWidget->setGstPipeline(pipeline);

    if (gstVideoWidget->isError()) {
        mError = MediaPlayerError;
        mErrorString = gstVideoWidget->errorString();
        delete gstVideoBoxWidget;
        return false;
    }

    gstVideoBoxWidget->setFixedSize(640, 360);

    int const index = static_cast<int>(mGstVideoBoxes.size());
    mMainLayout->addWidget(gstVideoBoxWidget, index / 2, index % 2, Qt::AlignCenter);

    mGstVideoBoxes.emplace(name, gstVideoBoxWidget);
    mError = NoError;

    return false;
}

auto GstVideoGridWidget::getGstVideoWidget(std::string const& name) -> GstVideoWidget* {
    if (!mGstVideoBoxes.contains(name)) {
        return nullptr;
    }
    return mGstVideoBoxes.at(name)->getGstVideoWidget();
}

auto GstVideoGridWidget::hideGstVideoWidget(std::string const& name) -> bool {
    if (!mGstVideoBoxes.contains(name)) {
        return false;
    }
    mGstVideoBoxes.at(name)->setVisible(false);
    return true;
}

auto GstVideoGridWidget::showGstVideoWidget(std::string const& name) -> bool {
    if (!mGstVideoBoxes.contains(name)) {
        return false;
    }
    mGstVideoBoxes.at(name)->setVisible(true);
    return true;
}

auto GstVideoGridWidget::error() const -> GstVideoGridWidget::Error {
    return mError;
}

auto GstVideoGridWidget::errorString() const -> QString {
    return mErrorString;
}

auto GstVideoGridWidget::isError() const -> bool {
    return mError != NoError;
}
