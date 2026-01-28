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

GstVideoWidget::GstVideoWidget(QWidget* parent) : QWidget(parent) {
}

GstVideoWidget::~GstVideoWidget() {
    if (mPipeline) {
        gst_element_set_state(mPipeline, GST_STATE_NULL);
        gst_object_unref(mPipeline);
    }
}

auto GstVideoWidget::setGstPipeline(std::string const& pipeline) -> void {
    if (mPipeline) {
        gst_element_set_state(mPipeline, GST_STATE_NULL);
        gst_object_unref(mPipeline);
        mPipeline = nullptr;
    }
    mPipelineString = pipeline + " ! videoconvert ! qwidget5videosink name=videosink";
    mStarted = false;
    mIsError = false;
    mErrorString.clear();
}

void GstVideoWidget::showEvent(QShowEvent* event) {
    QWidget::showEvent(event);
    if (mStarted || mPipelineString.empty()) return;
    mStarted = true;

    startPipeline();
}

void GstVideoWidget::startPipeline() {
    GError* err = nullptr;
    mPipeline = gst_parse_launch(mPipelineString.c_str(), &err);
    if (!mPipeline) {
        mIsError = true;
        mErrorString = err ? QString::fromUtf8(err->message) : "Failed to create pipeline";
        if (err) g_error_free(err);
        return;
    }
    if (err) g_error_free(err);

    GstElement* sink = gst_bin_get_by_name(GST_BIN(mPipeline), "videosink");
    if (sink) {
        g_object_set(sink, "widget", this, nullptr);
        gst_object_unref(sink);
    }

    play();
}

auto GstVideoWidget::errorString() const -> QString {
    return mErrorString;
}

auto GstVideoWidget::isError() const -> bool {
    return mIsError;
}

auto GstVideoWidget::play() -> void {
    if (mPipeline) gst_element_set_state(mPipeline, GST_STATE_PLAYING);
}

auto GstVideoWidget::pause() -> void {
    if (mPipeline) gst_element_set_state(mPipeline, GST_STATE_PAUSED);
}

auto GstVideoWidget::stop() -> void {
    if (mPipeline) gst_element_set_state(mPipeline, GST_STATE_NULL);
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
