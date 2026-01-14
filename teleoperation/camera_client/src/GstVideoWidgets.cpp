#include "GstVideoWidgets.hpp"

using namespace mrover;

GstVideoWidget::GstVideoWidget(QWidget* parent) : QVideoWidget(parent) {
    mPlayer = new QMediaPlayer(this);
    mPlayer->setVideoOutput(this);
}

auto GstVideoWidget::setGstPipeline(std::string const& pipeline) -> void {
    mPlayer->setMedia(QUrl(std::format("gst-pipeline: {} ! videoconvert ! xvimagesink name=\"qtvideosink\" sync=false", pipeline).c_str()));
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

GstVideoGridWidget::GstVideoGridWidget(QWidget* parent)
    : QWidget(parent), mError(NoError) {
    mMainLayout = new QGridLayout(this);
    mMainLayout->setContentsMargins(5, -1, 5, -1);
    mMainLayout->setSpacing(10);
    setLayout(mMainLayout);
}

auto GstVideoGridWidget::addGstVideoWidget(std::string const& name, std::string const& pipeline) -> bool {
    if (mGstVideoBoxes.contains(name)) {
        mError = ExistsError;
        mErrorString = "Camera name already exists";
        return false;
    }

    auto* gstVideoBoxWidget = new QWidget(this);
    auto* gstVideoBoxLayout = new QVBoxLayout(gstVideoBoxWidget);
    auto* gstVideoBoxLabel = new QLabel(QString::fromStdString(name), gstVideoBoxWidget);
    auto* gstVideoBoxGstVideoWidget = new GstVideoWidget(gstVideoBoxWidget);

    gstVideoBoxWidget->setMinimumSize(640, 480);

    gstVideoBoxGstVideoWidget->setGstPipeline(pipeline);
    if (gstVideoBoxGstVideoWidget->isError()) {
        mError = MediaPlayerError;
        mErrorString = gstVideoBoxGstVideoWidget->errorString();
        delete gstVideoBoxWidget;
        return false;
    }

    gstVideoBoxLayout->addWidget(gstVideoBoxLabel, 0);
    gstVideoBoxLayout->addWidget(gstVideoBoxGstVideoWidget, 1);
    gstVideoBoxWidget->setLayout(gstVideoBoxLayout);


    int const index = static_cast<int>(mGstVideoBoxes.size());
    mMainLayout->addWidget(gstVideoBoxWidget, index / 2, index % 2, Qt::AlignCenter);

    mGstVideoBoxes.emplace(name, GstVideoBox{.widget = gstVideoBoxWidget,
                                             .layout = gstVideoBoxLayout,
                                             .label = gstVideoBoxLabel,
                                             .gstVideoWidget = gstVideoBoxGstVideoWidget});
    clearError();
    return true;
}

auto GstVideoGridWidget::findVideoBox(std::string const& name) -> GstVideoBox* {
    auto it = mGstVideoBoxes.find(name);
    if (it == mGstVideoBoxes.end()) {
        setError(NonExistsError, "Camera name does not exist");
        return nullptr;
    }
    clearError();
    return &it->second;
}

auto GstVideoGridWidget::getGstVideoWidget(std::string const& name) -> GstVideoWidget* {
    auto* box = findVideoBox(name);
    return box ? box->gstVideoWidget : nullptr;
}

auto GstVideoGridWidget::playVideo(std::string const& name) -> bool {
    auto* box = findVideoBox(name);
    if (!box) return false;
    box->gstVideoWidget->play();
    return true;
}

auto GstVideoGridWidget::pauseVideo(std::string const& name) -> bool {
    auto* box = findVideoBox(name);
    if (!box) return false;
    box->gstVideoWidget->pause();
    return true;
}

auto GstVideoGridWidget::stopVideo(std::string const& name) -> bool {
    auto* box = findVideoBox(name);
    if (!box) return false;
    box->gstVideoWidget->stop();
    return true;
}

auto GstVideoGridWidget::hideVideo(std::string const& name) -> bool {
    auto* box = findVideoBox(name);
    if (!box) return false;
    box->widget->setVisible(false);
    return true;
}

auto GstVideoGridWidget::showVideo(std::string const& name) -> bool {
    auto* box = findVideoBox(name);
    if (!box) return false;
    box->widget->setVisible(true);
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

auto GstVideoGridWidget::clearError() -> void {
    mError = NoError;
    mErrorString.clear();
}

auto GstVideoGridWidget::setError(Error error, std::string const& errorString) -> void {
    mError = error;
    mErrorString = QString::fromStdString(errorString);
}
