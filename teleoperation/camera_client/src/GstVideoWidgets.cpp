#include "GstVideoWidgets.hpp"

using namespace mrover;

VideoSurface::VideoSurface(GstVideoWidget* widget, QObject* parent)
    : QAbstractVideoSurface(parent), mWidget(widget) {}

QList<QVideoFrame::PixelFormat> VideoSurface::supportedPixelFormats(QAbstractVideoBuffer::HandleType handleType) const {
    if (handleType == QAbstractVideoBuffer::NoHandle) {
        return {
            QVideoFrame::Format_RGB32,
            QVideoFrame::Format_ARGB32,
            QVideoFrame::Format_RGB24,
            QVideoFrame::Format_BGR32,
            QVideoFrame::Format_BGR24
        };
    }
    return {};
}

bool VideoSurface::present(const QVideoFrame& frame) {
    if (frame.isValid()) {
        QVideoFrame cloneFrame(frame);
        if (cloneFrame.map(QAbstractVideoBuffer::ReadOnly)) {
            QImage image(cloneFrame.bits(), cloneFrame.width(), cloneFrame.height(),
                         cloneFrame.bytesPerLine(), QVideoFrame::imageFormatFromPixelFormat(cloneFrame.pixelFormat()));
            // image.copy() is required: cloneFrame.bits() becomes invalid after unmap
            mWidget->updateFrame(image.copy());
            cloneFrame.unmap();
        }
    }
    return true;
}

GstVideoWidget::GstVideoWidget(QWidget* parent) : QWidget(parent) {
    mPlayer = new QMediaPlayer(this);
    mSurface = new VideoSurface(this, this);
    mPlayer->setVideoOutput(mSurface);
    setMouseTracking(true);
}

auto GstVideoWidget::setGstPipeline(std::string const& pipeline) -> void {
    // qtvideosink is required here (not xvimagesink) so VideoSurface::present receives frames
    mPlayer->setMedia(QUrl(QString::fromStdString(
        std::format("gst-pipeline: {} ! videoconvert ! qtvideosink name=\"qtvideosink\" sync=false", pipeline))));
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

void GstVideoWidget::sampleColorAtCursor() {
    if (mLastFrame.isNull() || width() == 0 || height() == 0) return;

    QPoint framePos(mLastCursorPos.x() * mLastFrame.width() / width(),
                    mLastCursorPos.y() * mLastFrame.height() / height());
    if (!mLastFrame.rect().contains(framePos)) return;

    mLastPickedColor = mLastFrame.pixelColor(framePos);
    emit colorPicked(mLastPickedColor);
}

void GstVideoWidget::updateFrame(const QImage& frame) {
    mLastFrame = frame;
    if (mMouseInWidget) sampleColorAtCursor();
    update();
}

void GstVideoWidget::paintEvent(QPaintEvent* event) {
    Q_UNUSED(event);
    QPainter painter(this);

    if (!mLastFrame.isNull()) {
        painter.drawImage(rect(), mLastFrame);
    } else {
        painter.fillRect(rect(), Qt::black);
        painter.setPen(Qt::white);
        painter.drawText(rect(), Qt::AlignCenter, "No Video Frame Received");
    }

    if (mMouseInWidget) {
        int x = mLastCursorPos.x();
        int y = mLastCursorPos.y();
        constexpr int arm = 8;

        painter.setPen(QPen(QColor(0, 0, 0, 180), 3));
        painter.drawLine(x - arm, y, x + arm, y);
        painter.drawLine(x, y - arm, x, y + arm);

        painter.setPen(QPen(Qt::white, 1));
        painter.drawLine(x - arm, y, x + arm, y);
        painter.drawLine(x, y - arm, x, y + arm);
    }
}

void GstVideoWidget::mouseMoveEvent(QMouseEvent* event) {
    mLastCursorPos = event->pos();
    if (mMouseInWidget) sampleColorAtCursor();
    update();
    QWidget::mouseMoveEvent(event);
}

void GstVideoWidget::enterEvent(QEvent* event) {
    mMouseInWidget = true;
    update();
    QWidget::enterEvent(event);
}

void GstVideoWidget::leaveEvent(QEvent* event) {
    mMouseInWidget = false;
    update();
    QWidget::leaveEvent(event);
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

    gstVideoBoxWidget->setMinimumSize(640, 360);

    gstVideoBoxGstVideoWidget->setGstPipeline(pipeline);
    if (gstVideoBoxGstVideoWidget->isError()) {
        mError = MediaPlayerError;
        mErrorString = gstVideoBoxGstVideoWidget->errorString();
        delete gstVideoBoxWidget;
        return false;
    }

    connect(gstVideoBoxGstVideoWidget, &GstVideoWidget::colorPicked,
            this, [this, name](QColor color) {
                emit colorPicked(QString::fromStdString(name), color);
            });

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

auto GstVideoGridWidget::getGstVideoWidget(std::string const& name) -> GstVideoWidget* {
    if (!mGstVideoBoxes.contains(name)) {
        return nullptr;
    }
    return mGstVideoBoxes.at(name).gstVideoWidget;
}

auto GstVideoGridWidget::playVideo(std::string const& name) -> bool {
    if (!mGstVideoBoxes.contains(name)) {
        setError(NonExistsError, "Camera name does not exist");
        return false;
    }
    mGstVideoBoxes.at(name).gstVideoWidget->play();

    clearError();
    return true;
}

auto GstVideoGridWidget::pauseVideo(std::string const& name) -> bool {
    if (!mGstVideoBoxes.contains(name)) {
        setError(NonExistsError, "Camera name does not exist");
        return false;
    }
    mGstVideoBoxes.at(name).gstVideoWidget->pause();

    clearError();
    return true;
}

auto GstVideoGridWidget::stopVideo(std::string const& name) -> bool {
    if (!mGstVideoBoxes.contains(name)) {
        setError(NonExistsError, "Camera name does not exist");
        return false;
    }
    mGstVideoBoxes.at(name).gstVideoWidget->stop();

    clearError();
    return true;
}

auto GstVideoGridWidget::hideVideo(std::string const& name) -> bool {
    if (!mGstVideoBoxes.contains(name)) {
        setError(NonExistsError, "Camera name does not exist");
        return false;
    }
    mGstVideoBoxes.at(name).widget->setVisible(false);

    clearError();
    return true;
}

auto GstVideoGridWidget::showVideo(std::string const& name) -> bool {
    if (!mGstVideoBoxes.contains(name)) {
        setError(NonExistsError, "Camera name does not exist");
        return false;
    }
    mGstVideoBoxes.at(name).widget->setVisible(true);

    clearError();
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
