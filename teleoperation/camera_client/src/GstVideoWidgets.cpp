#include "GstVideoWidgets.hpp"

using namespace mrover;

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

    // Check if we've moved far enough to start a drag
    if ((event->pos() - mDragStartPosition).manhattanLength() < QApplication::startDragDistance()) {
        return;
    }

    auto* drag = new QDrag(this);
    auto* mimeData = new QMimeData();
    mimeData->setText(QString::fromStdString(mCameraName));
    drag->setMimeData(mimeData);

    // Create a semi-transparent pixmap for visual feedback
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
// GstVideoGridWidget
// --------------------------------------------

GstVideoGridWidget::GstVideoGridWidget(QWidget* parent)
    : QWidget(parent), mError(NoError) {
    mMainLayout = new QGridLayout(this);
    mMainLayout->setContentsMargins(5, -1, 5, -1);
    mMainLayout->setSpacing(10);
    setLayout(mMainLayout);
    setAcceptDrops(true);
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

    gstVideoBoxLayout->addWidget(gstVideoBoxLabel, 0);
    gstVideoBoxLayout->addWidget(gstVideoBoxGstVideoWidget, 1);
    gstVideoBoxWidget->setLayout(gstVideoBoxLayout);

    mGstVideoBoxes.emplace(name, GstVideoBox{.widget = gstVideoBoxWidget,
                                             .layout = gstVideoBoxLayout,
                                             .label = gstVideoBoxLabel,
                                             .gstVideoWidget = gstVideoBoxGstVideoWidget});

    mVisibleOrder.push_back(name);
    rebuildGrid();

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
    // Remove all widgets from the grid (without deleting them)
    for (auto& [name, box]: mGstVideoBoxes) {
        mMainLayout->removeWidget(box.widget);
    }

    // Re-add only visible widgets in order
    for (std::size_t idx = 0; idx < mVisibleOrder.size(); ++idx) {
        auto* box = findVideoBox(mVisibleOrder[idx]);
        if (box) {
            int const row = static_cast<int>(idx) / 2;
            int const col = static_cast<int>(idx) % 2;
            mMainLayout->addWidget(box->widget, row, col, Qt::AlignCenter);
            box->widget->setVisible(true);
        }
    }
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

    // Remove from visible order
    auto it = std::find(mVisibleOrder.begin(), mVisibleOrder.end(), name);
    if (it != mVisibleOrder.end()) {
        mVisibleOrder.erase(it);
    }

    box->widget->setVisible(false);
    rebuildGrid();
    return true;
}

auto GstVideoGridWidget::showVideo(std::string const& name) -> bool {
    auto* box = findVideoBox(name);
    if (!box) return false;

    // Add to end of visible order (if not already visible)
    auto it = std::find(mVisibleOrder.begin(), mVisibleOrder.end(), name);
    if (it == mVisibleOrder.end()) {
        mVisibleOrder.push_back(name);
    }

    rebuildGrid();
    return true;
}

auto GstVideoGridWidget::moveCamera(std::string const& name, int newIndex) -> bool {
    // Find current position in visible order
    auto it = std::find(mVisibleOrder.begin(), mVisibleOrder.end(), name);
    if (it == mVisibleOrder.end()) {
        setError(NonExistsError, "Camera is not visible");
        return false;
    }

    // Clamp newIndex to valid range
    int const maxIndex = static_cast<int>(mVisibleOrder.size()) - 1;
    int const clampedIndex = std::clamp(newIndex, 0, maxIndex);

    // Remove from current position and insert at new position
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

    // Get the size of the first visible widget to estimate cell dimensions
    auto it = mGstVideoBoxes.find(mVisibleOrder.front());
    if (it == mGstVideoBoxes.end()) {
        return static_cast<int>(mVisibleOrder.size());
    }

    QSize const cellSize = it->second.widget->size();
    int const spacing = mMainLayout->spacing();

    int const col = pos.x() / (cellSize.width() + spacing);
    int const row = pos.y() / (cellSize.height() + spacing);
    int const index = row * 2 + col;

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
