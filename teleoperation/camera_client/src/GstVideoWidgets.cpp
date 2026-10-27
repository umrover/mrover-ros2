#include "GstVideoWidgets.hpp"
#include <gst/app/gstappsink.h>
#include <gst/gstelement.h>
#include <gst/video/video.h>
#include <qglobal.h>
#include <qwidget.h>

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
    }
    if (mAppSink) {
        gst_object_unref(mAppSink);
    }
    if (mPipeline) {
        gst_object_unref(mPipeline);
    }
}

auto GstVideoWidget::setGstPipeline(std::string const& pipeline) -> void {
    mBasePipeline = pipeline;
    applyPipeline();
}

auto GstVideoWidget::onNewSample(GstAppSink* sink, gpointer userData) -> GstFlowReturn {
    auto* widget = static_cast<GstVideoWidget*>(userData);
    GstSample* sample = gst_app_sink_pull_sample(sink);

    if (!sample) {
        return GST_FLOW_EOS;
    }

    GstCaps* caps = gst_sample_get_caps(sample);
    GstVideoInfo info;
    gst_video_info_init(&info);
    if (!caps || !gst_video_info_from_caps(&info, caps)) {
        qWarning() << "GStreamer appsink received a sample with invalid video caps";
        gst_sample_unref(sample);
        return GST_FLOW_ERROR;
    }

    GstVideoFrame videoFrame;
    if (!gst_video_frame_map(&videoFrame, &info, gst_sample_get_buffer(sample), GST_MAP_READ)) {
        qWarning() << "Failed to map GStreamer video frame";
        gst_sample_unref(sample);
        return GST_FLOW_ERROR;
    }

    QImage image(
            static_cast<uchar const*>(GST_VIDEO_FRAME_PLANE_DATA(&videoFrame, 0)),
            static_cast<int>(GST_VIDEO_INFO_WIDTH(&info)),
            static_cast<int>(GST_VIDEO_INFO_HEIGHT(&info)),
            GST_VIDEO_FRAME_PLANE_STRIDE(&videoFrame, 0),
            QImage::Format_RGB888);
    QImage frameCopy = image.copy();

    gst_video_frame_unmap(&videoFrame);
    gst_sample_unref(sample);

    if (frameCopy.isNull()) {
        qWarning() << "Failed to copy GStreamer video frame";
        return GST_FLOW_ERROR;
    }

    QMetaObject::invokeMethod(
            widget,
            [widget, frame = std::move(frameCopy)]() mutable {
                widget->mFrame = std::move(frame);
                widget->update();
            },
            Qt::QueuedConnection);

    return GST_FLOW_OK;
}

auto GstVideoWidget::applyPipeline() -> void {
    std::string flipElement;
    if (mRotation == 1) {
        flipElement = " ! videoflip method=clockwise";
    } else if (mRotation == 2) {
        flipElement = " ! videoflip method=rotate-180";
    } else if (mRotation == 3) {
        flipElement = " ! videoflip method=counterclockwise";
    }

    if (mPipeline) {
        gst_element_set_state(mPipeline, GST_STATE_NULL);
        if (mAppSink) {
            gst_object_unref(mAppSink);
            mAppSink = nullptr;
        }
        gst_object_unref(mPipeline);
        mPipeline = nullptr;
    }

    mErrorString.clear();
    mFrame = {};
    update();

    std::string const launch = std::format("{}{} ! videoconvert ! video/x-raw,format=RGB ! appsink name=video_sink emit-signals=true sync=false max-buffers=1 drop=true", mBasePipeline, flipElement);

    GError* error = nullptr;
    mPipeline = gst_parse_launch(launch.c_str(), &error);
    if (error) {
        qWarning() << "GStreamer pipeline error:" << error->message;
        mErrorString = QString::fromUtf8(error->message);
        g_error_free(error);
        if (mPipeline) {
            gst_object_unref(mPipeline);
            mPipeline = nullptr;
        }
        return;
    }
    if (!mPipeline) {
        mErrorString = "Failed to create GStreamer pipeline";
        qWarning() << mErrorString;
        return;
    }

    mAppSink = GST_APP_SINK(gst_bin_get_by_name(GST_BIN(mPipeline), "video_sink"));
    if (!mAppSink) {
        mErrorString = "Pipeline has no video_sink appsink";
        qWarning() << mErrorString;
        gst_object_unref(mPipeline);
        mPipeline = nullptr;
        return;
    }

    g_signal_connect(mAppSink, "new-sample", G_CALLBACK(&GstVideoWidget::onNewSample), this);

    if (gst_element_set_state(mPipeline, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
        mErrorString = "Failed to start GStreamer pipeline";
        qWarning() << mErrorString;
    }
}

auto GstVideoWidget::rotate90() -> void {
    mRotation = (mRotation + 1) % 4;
    applyPipeline();
}

auto GstVideoWidget::errorString() const -> QString {
    return mErrorString;
}

auto GstVideoWidget::isError() const -> bool {
    return !mErrorString.isEmpty();
}

auto GstVideoWidget::play() -> void {
    qDebug() << "Request to play a pipeline";

    if (mPipeline && gst_element_set_state(mPipeline, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
        mErrorString = "Failed to play GStreamer pipeline";
        qWarning() << mErrorString;
    }
}
auto GstVideoWidget::pause() -> void {
    qDebug() << "Request to pause a pipeline";

    if (mPipeline && gst_element_set_state(mPipeline, GST_STATE_PAUSED) == GST_STATE_CHANGE_FAILURE) {
        mErrorString = "Failed to pause GStreamer pipeline";
        qWarning() << mErrorString;
    }
}

auto GstVideoWidget::stop() -> void {
    qDebug() << "Request to stop a pipeline";

    if (mPipeline && gst_element_set_state(mPipeline, GST_STATE_NULL) == GST_STATE_CHANGE_FAILURE) {
        mErrorString = "Failed to stop GStreamer pipeline";
        qWarning() << mErrorString;
    }
}

void GstVideoWidget::paintEvent(QPaintEvent* event) {
    QWidget::paintEvent(event);

    QPainter painter(this);
    painter.fillRect(rect(), Qt::black);

    if (mFrame.isNull()) {
        return;
    }

    QSize scaledSize = mFrame.size();
    scaledSize.scale(size(), Qt::KeepAspectRatio);

    QRect target(
            QPoint((width() - scaledSize.width()) / 2,
                   (height() - scaledSize.height()) / 2),
            scaledSize);
    painter.drawImage(target, mFrame);
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

    int currentVideoWidth = MIN_VIDEO_WIDTH;
    if (!mVisibleOrder.empty()) {
        auto it = mGstVideoBoxes.find(mVisibleOrder.front());
        if (it != mGstVideoBoxes.end()) {
            currentVideoWidth = it->second.widget->width();
        }
    }

    int const maxColumns = std::max(1, availableWidth / (currentVideoWidth + mMainLayout->spacing()));
    int const visibleCount = std::max(1, static_cast<int>(mVisibleOrder.size()));
    int const idealColumns = static_cast<int>(std::ceil(std::sqrt(visibleCount)));

    return std::min(maxColumns, idealColumns);
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
    gstVideoBoxWidget->setMaximumSize(640, 480);

    gstVideoBoxGstVideoWidget->setGstPipeline(pipeline);
    if (gstVideoBoxGstVideoWidget->isError()) {
        mError = MediaPipelineError;
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

auto GstVideoGridWidget::rotateCamera(std::string const& name) -> bool {
    auto* box = findVideoBox(name);
    if (!box) return false;

    box->gstVideoWidget->rotate90();

    QSize const currentSize = box->widget->minimumSize();
    int const newWidth = currentSize.height();
    int const newHeight = currentSize.width();

    box->widget->setMinimumSize(newWidth, newHeight);
    box->widget->setMaximumSize(newWidth, newHeight);
    box->widget->resize(newWidth, newHeight);

    rebuildGrid();

    return true;
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

auto GstVideoGridWidget::resizeCamera(std::string const& name, int width, int height) -> bool {
    auto* box = findVideoBox(name);
    if (!box) return false;
    box->widget->setMinimumSize(width, height);
    box->widget->setMaximumSize(width, height);
    box->widget->resize(width, height);
    rebuildGrid();
    return true;
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
