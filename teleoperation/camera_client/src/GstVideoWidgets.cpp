#include "GstVideoWidgets.hpp"
#include <gst/app/gstappsink.h>
#include <gst/gstelement.h>
#include <gst/video/video.h>
#include <qdebug.h>
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
    if (mAppSink) {
        gst_object_unref(mAppSink);
    }
}

auto GstVideoWidget::setGstPipeline(std::string const& pipeline) -> void {
    mBasePipeline = pipeline;
    applyPipeline();
}

auto GstVideoWidget::onNewSample(GstAppSink* sink, GstVideoWidget* userData) -> GstFlowReturn {
    GstSample* sample = gst_app_sink_pull_sample(sink);

    if (!sample) {
        qInfo() << "A stream ended";
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
        qWarning() << "Failed to map a GStreamer video frame";
        gst_sample_unref(sample);
        return GST_FLOW_ERROR;
    }

    QImage image(
            static_cast<uchar const*>(GST_VIDEO_FRAME_PLANE_DATA(&videoFrame, 0)),
            static_cast<int>(GST_VIDEO_INFO_WIDTH(&info)),
            static_cast<int>(GST_VIDEO_INFO_HEIGHT(&info)),
            GST_VIDEO_FRAME_PLANE_STRIDE(&videoFrame, 0),
            QImage::Format_RGB888);

    gst_video_frame_unmap(&videoFrame);
    gst_sample_unref(sample);

    if (image.isNull()) {
        qWarning() << "Failed to copy a GStreamer video frame";
        return GST_FLOW_ERROR;
    }

    QMetaObject::invokeMethod(
            userData,
            [userData, frame = std::move(image)]() mutable {
                userData->mFrame = std::move(frame);
                userData->update();
            },
            Qt::QueuedConnection);

    return GST_FLOW_OK;
}

auto GstVideoWidget::applyPipeline() -> void {
    gst::PipelineBuilder pipeline;

    pipeline.pushBack(mBasePipeline);
    pipeline.pushBack("videoconvert");
    pipeline.pushBack("video/x-raw,format=RGB");
    pipeline.pushBack("appsink name=video_sink emit-signals=true sync=false max-buffers=1 drop=true");
    mErrorString.clear();
    mFrame = {};
    update();

    // start pipeline
    mStreamPipelineWrapper = gst::PipelineWrapper(pipeline.str());
    try {
        mStreamPipelineWrapper.init();
    } catch (std::runtime_error const& e) {
        mErrorString = QString::fromStdString(std::format("Failed to create GStreamer pipeline: {}", e.what()));
        qWarning() << mErrorString;
        return;
    }

    // create sink

    mAppSink = GST_APP_SINK(gst_bin_get_by_name(GST_BIN(mStreamPipelineWrapper.getPipelineElement()), "video_sink"));
    if (!mAppSink) {
        mErrorString = QString::fromStdString("Pipeline '" + mBasePipeline + "' has no video_sink appsink");
        qWarning() << mErrorString;
        mStreamPipelineWrapper.stop();
        return;
    }

    g_signal_connect(mAppSink, "new-sample", G_CALLBACK(&GstVideoWidget::onNewSample), this);

    play();
}

auto GstVideoWidget::rotate90() -> void {
    mRotation = (mRotation + 1) % 4;
    update();
}

auto GstVideoWidget::errorString() const -> QString {
    return mErrorString;
}

auto GstVideoWidget::isError() const -> bool {
    return !mErrorString.isEmpty();
}

auto GstVideoWidget::play() -> void {
    qDebug() << "Request to play a pipeline";

    try {
        mStreamPipelineWrapper.play();
    } catch (std::runtime_error const& e) {
        mErrorString = QString::fromStdString(std::format("Failed to play GStreamer pipeline: {}", e.what()));
        qWarning() << mErrorString;
        return;
    }
}
auto GstVideoWidget::pause() -> void {
    qDebug() << "Request to pause a pipeline";

    try {
        mStreamPipelineWrapper.pause();
    } catch (std::runtime_error const& e) {
        mErrorString = QString::fromStdString(std::format("Failed to pause GStreamer pipeline: {}", e.what()));
        qWarning() << mErrorString;
        return;
    }
}

// Technically just pauses the stream for now
auto GstVideoWidget::stop() -> void {
    qDebug() << "Request to stop a pipeline";

    try {
        mStreamPipelineWrapper.pause();
    } catch (std::runtime_error const& e) {
        mErrorString = QString::fromStdString(std::format("Failed to stop GStreamer pipeline: {}", e.what()));
        qWarning() << mErrorString;
        return;
    }
}

void GstVideoWidget::paintEvent(QPaintEvent* event) {
    QWidget::paintEvent(event);

    QPainter painter(this);
    painter.fillRect(rect(), Qt::black);

    if (mFrame.isNull()) {
        return;
    }

    QSize orientedSize = mFrame.size();
    if (mRotation % 2 != 0) {
        orientedSize.transpose();
    }

    qreal const scale = std::min(static_cast<qreal>(width()) / orientedSize.width(),
                                 static_cast<qreal>(height()) / orientedSize.height());
    qreal const drawWidth = mFrame.width() * scale;
    qreal const drawHeight = mFrame.height() * scale;

    painter.translate(width() / 2.0, height() / 2.0);
    painter.rotate(mRotation * 90.0);

    QRectF target(
            -drawWidth / 2.0,
            -drawHeight / 2.0,
            drawWidth,
            drawHeight);

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
