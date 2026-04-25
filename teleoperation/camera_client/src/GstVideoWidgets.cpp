#include "GstVideoWidgets.hpp"

using namespace mrover;

<<<<<<< HEAD
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
=======
// --------------------------------------------
// DraggableVideoFrame
// --------------------------------------------

DraggableVideoFrame::DraggableVideoFrame(std::string cameraName, QWidget* parent)
    : QFrame(parent), mCameraName(std::move(cameraName)) {
    setFrameShape(QFrame::StyledPanel);
    setFrameShadow(QFrame::Raised);
    setCursor(Qt::OpenHandCursor);
}

void DraggableVideoFrame::mousePressEvent(QMouseEvent* event) {
    if (event->button() == Qt::LeftButton) {
        mDragStartPosition = event->pos();
        setCursor(Qt::ClosedHandCursor);
    }
    QFrame::mousePressEvent(event);
}

void DraggableVideoFrame::mouseMoveEvent(QMouseEvent* event) {
    if (!(event->buttons() & Qt::LeftButton)) {
        return;
    }

    // check if we've moved far enough to start a drag
    if ((event->pos() - mDragStartPosition).manhattanLength() < QApplication::startDragDistance()) {
        return;
    }

    auto* drag = new QDrag(this);
    auto* mimeData = new QMimeData();
    mimeData->setText(QString::fromStdString(mCameraName));
    drag->setMimeData(mimeData);

    // visual feedback
    QPixmap pixmap = grab();
    pixmap = pixmap.scaled(pixmap.size() / 2, Qt::KeepAspectRatio, Qt::SmoothTransformation);
    drag->setPixmap(pixmap);
    drag->setHotSpot(QPoint(pixmap.width() / 2, pixmap.height() / 2));

    setCursor(Qt::OpenHandCursor);
    drag->exec(Qt::MoveAction);
}

// --------------------------------------------
// GstVideoWidget
// --------------------------------------------

GstVideoWidget::GstVideoWidget(QWidget* parent) : QVideoWidget(parent) {
>>>>>>> origin/main
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
// GstVideoGridWidget
// --------------------------------------------

namespace {
    constexpr int MIN_VIDEO_WIDTH = 640;
}

GstVideoGridWidget::GstVideoGridWidget(QWidget* parent)
    : QWidget(parent), mError(NoError) {
    mMainLayout = new QGridLayout(this);
    mMainLayout->setContentsMargins(5, 5, 5, 5);
    mMainLayout->setSpacing(10);
    setLayout(mMainLayout);
    setAcceptDrops(true);
}

auto GstVideoGridWidget::calculateColumnCount() const -> int {
    int const availableWidth = width() - mMainLayout->contentsMargins().left() - mMainLayout->contentsMargins().right();
    int const cellWidth = MIN_VIDEO_WIDTH + mMainLayout->spacing();
    int const columns = std::max(1, availableWidth / cellWidth);
    return columns;
}

auto GstVideoGridWidget::addGstVideoWidget(std::string const& name, std::string const& pipeline) -> bool {
    if (mGstVideoBoxes.contains(name)) {
        mError = ExistsError;
        mErrorString = "Camera name already exists";
        return false;
    }

    auto* gstVideoBoxWidget = new DraggableVideoFrame(name, this);
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

    connect(gstVideoBoxGstVideoWidget, &GstVideoWidget::colorPicked,
            this, [this, name](QColor color) {
                emit colorPicked(QString::fromStdString(name), color);
            });

    gstVideoBoxLayout->addWidget(gstVideoBoxLabel, 0);
    gstVideoBoxLayout->addWidget(gstVideoBoxGstVideoWidget, 1);
    gstVideoBoxWidget->setLayout(gstVideoBoxLayout);

<<<<<<< HEAD
    int const index = static_cast<int>(mGstVideoBoxes.size());
    mMainLayout->addWidget(gstVideoBoxWidget, index / 2, index % 2, Qt::AlignCenter);

=======
>>>>>>> origin/main
    mGstVideoBoxes.emplace(name, GstVideoBox{.widget = gstVideoBoxWidget,
                                             .layout = gstVideoBoxLayout,
                                             .label = gstVideoBoxLabel,
                                             .gstVideoWidget = gstVideoBoxGstVideoWidget});

    mVisibleOrder.push_back(name);
    rebuildGrid();

    hideAll();

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

auto GstVideoGridWidget::rebuildGrid() -> void {
    // remove all widgets from the grid (without deleting them)
    for (auto& [name, box]: mGstVideoBoxes) {
        mMainLayout->removeWidget(box.widget);
    }

    int const columns = calculateColumnCount();

    // re-add only visible widgets in order
    for (std::size_t idx = 0; idx < mVisibleOrder.size(); ++idx) {
        auto* box = findVideoBox(mVisibleOrder[idx]);
        if (box) {
            int const row = static_cast<int>(idx) / columns;
            int const col = static_cast<int>(idx) % columns;
            mMainLayout->addWidget(box->widget, row, col, Qt::AlignCenter);
            box->widget->setVisible(true);
        }
    }
}

void GstVideoGridWidget::resizeEvent(QResizeEvent* event) {
    QWidget::resizeEvent(event);
    rebuildGrid();
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

    auto it = std::find(mVisibleOrder.begin(), mVisibleOrder.end(), name);
    if (it != mVisibleOrder.end()) {
        mVisibleOrder.erase(it);
    }

    box->widget->setVisible(false);
    rebuildGrid();
    return true;
}

auto GstVideoGridWidget::hideAll() -> void {
    for (auto const& [name, _]: mGstVideoBoxes) {
        hideVideo(name);
    }
}

auto GstVideoGridWidget::showVideo(std::string const& name) -> bool {
    auto* box = findVideoBox(name);
    if (!box) return false;

    auto it = std::find(mVisibleOrder.begin(), mVisibleOrder.end(), name);
    if (it == mVisibleOrder.end()) {
        mVisibleOrder.push_back(name);
    }

    rebuildGrid();
    return true;
}

auto GstVideoGridWidget::moveCamera(std::string const& name, int newIndex) -> bool {
    auto it = std::find(mVisibleOrder.begin(), mVisibleOrder.end(), name);
    if (it == mVisibleOrder.end()) {
        setError(NonExistsError, "Camera is not visible");
        return false;
    }

    int const maxIndex = static_cast<int>(mVisibleOrder.size()) - 1;
    int const clampedIndex = std::clamp(newIndex, 0, maxIndex);

    mVisibleOrder.erase(it);
    mVisibleOrder.insert(mVisibleOrder.begin() + clampedIndex, name);

    rebuildGrid();
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

auto GstVideoGridWidget::getDropTargetIndex(QPoint const& pos) const -> int {
    if (mVisibleOrder.empty()) {
        return 0;
    }

    auto it = mGstVideoBoxes.find(mVisibleOrder.front());
    if (it == mGstVideoBoxes.end()) {
        return static_cast<int>(mVisibleOrder.size());
    }

    QSize const cellSize = it->second.widget->size();
    int const spacing = mMainLayout->spacing();
    int const columns = calculateColumnCount();

    int const col = pos.x() / (cellSize.width() + spacing);
    int const row = pos.y() / (cellSize.height() + spacing);
    int const index = row * columns + col;

    return std::clamp(index, 0, static_cast<int>(mVisibleOrder.size()));
}

void GstVideoGridWidget::dragEnterEvent(QDragEnterEvent* event) {
    if (event->mimeData()->hasText()) {
        std::string const cameraName = event->mimeData()->text().toStdString();
        if (mGstVideoBoxes.contains(cameraName)) {
            event->acceptProposedAction();
            return;
        }
    }
    event->ignore();
}

void GstVideoGridWidget::dragMoveEvent(QDragMoveEvent* event) {
    if (event->mimeData()->hasText()) {
        event->acceptProposedAction();
    }
}

void GstVideoGridWidget::dropEvent(QDropEvent* event) {
    if (!event->mimeData()->hasText()) {
        event->ignore();
        return;
    }

    std::string const cameraName = event->mimeData()->text().toStdString();
    int const targetIndex = getDropTargetIndex(event->pos());

    if (moveCamera(cameraName, targetIndex)) {
        event->acceptProposedAction();
    } else {
        event->ignore();
    }
}
