#pragma once

#include "pch.hpp"

namespace mrover {

    class DraggableVideoFrame : public QFrame {
        Q_OBJECT

        std::string mCameraName;
        QPoint mDragStartPosition;

    protected:
        void mousePressEvent(QMouseEvent* event) override;
        void mouseMoveEvent(QMouseEvent* event) override;

    public:
        explicit DraggableVideoFrame(std::string cameraName, QWidget* parent = nullptr);

        [[nodiscard]] auto cameraName() const -> std::string const& { return mCameraName; }
    };

    class GstVideoWidget;

    class VideoSurface : public QAbstractVideoSurface {
        Q_OBJECT
        GstVideoWidget* mWidget;

    public:
        explicit VideoSurface(GstVideoWidget* widget, QObject* parent = nullptr);

        QList<QVideoFrame::PixelFormat> supportedPixelFormats(
            QAbstractVideoBuffer::HandleType handleType) const override;

        bool present(const QVideoFrame& frame) override;
    };

    class GstVideoWidget : public QWidget {
        Q_OBJECT

        QMediaPlayer* mPlayer;
        VideoSurface* mSurface;
        QImage mLastFrame;

        QColor mLastPickedColor;
        QPoint mLastCursorPos;
        bool mMouseInWidget = false;

        void sampleColorAtCursor();

    public:
        explicit GstVideoWidget(QWidget* parent = nullptr);

        auto setGstPipeline(std::string const& pipeline) -> void;

        [[nodiscard]] auto errorString() const -> QString;
        [[nodiscard]] auto error() const -> QMediaPlayer::Error;
        [[nodiscard]] auto isError() const -> bool;

        auto play() -> void;
        auto pause() -> void;
        auto stop() -> void;

        void updateFrame(const QImage& frame);

    protected:
        void mouseMoveEvent(QMouseEvent* event) override;
        void enterEvent(QEvent* event) override;
        void leaveEvent(QEvent* event) override;
        void paintEvent(QPaintEvent* event) override;

    signals:
        void colorPicked(QColor color);
    };

    class GstVideoGridWidget : public QWidget {
        Q_OBJECT
    public:
        enum Error {
            NoError,
            NonExistsError,
            ExistsError,
            MediaPlayerError,
        };

    private:
        struct GstVideoBox {
            DraggableVideoFrame* widget;
            QVBoxLayout* layout;

            QLabel* label;
            GstVideoWidget* gstVideoWidget;
        };

        QGridLayout* mMainLayout;
        std::unordered_map<std::string, GstVideoBox> mGstVideoBoxes;
        std::vector<std::string> mVisibleOrder;

        GstVideoGridWidget::Error mError;
        QString mErrorString;

        auto clearError() -> void;
        auto setError(Error error, std::string const& errorString) -> void;
        auto findVideoBox(std::string const& name) -> GstVideoBox*;
        auto rebuildGrid() -> void;
        auto getDropTargetIndex(QPoint const& pos) const -> int;
        auto calculateColumnCount() const -> int;

    protected:
        void dragEnterEvent(QDragEnterEvent* event) override;
        void dragMoveEvent(QDragMoveEvent* event) override;
        void dropEvent(QDropEvent* event) override;
        void resizeEvent(QResizeEvent* event) override;

    public:
        explicit GstVideoGridWidget(QWidget* parent = nullptr);

        auto addGstVideoWidget(std::string const& name, std::string const& pipeline) -> bool;
        auto getGstVideoWidget(std::string const& name) -> GstVideoWidget*;
        auto playVideo(std::string const& name) -> bool;
        auto pauseVideo(std::string const& name) -> bool;
        auto stopVideo(std::string const& name) -> bool;
        auto hideVideo(std::string const& name) -> bool;
        auto showVideo(std::string const& name) -> bool;
        auto moveCamera(std::string const& name, int newIndex) -> bool;
        auto hideAll() -> void;

        [[nodiscard]] auto error() const -> GstVideoGridWidget::Error;
        [[nodiscard]] auto errorString() const -> QString;
        [[nodiscard]] auto isError() const -> bool;

    signals:
        void colorPicked(QString cameraName, QColor color);
    };

} // namespace mrover
