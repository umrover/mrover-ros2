#pragma once

#include "pch.hpp"

namespace mrover {
    class GstVideoWidget;

    class VideoSurface : public QAbstractVideoSurface {
        Q_OBJECT
        GstVideoWidget* mWidget;

    public:
        explicit VideoSurface(GstVideoWidget* widget, QObject* parent = nullptr);

        QList<QVideoFrame::PixelFormat> supportedPixelFormats(
            QAbstractVideoBuffer::HandleType handleType = QAbstractVideoBuffer::NoHandle) const override;

        bool present(const QVideoFrame& frame) override;
    };

    class GstVideoWidget : public QWidget {
        Q_OBJECT

        QMediaPlayer* mPlayer;
        VideoSurface* mSurface;
        QImage mLastFrame;

        QColor mLastPickedColor;
        QPoint mLastCursorPos;
        bool mShowColorOverlay = false;

    public:
        explicit GstVideoWidget(QWidget* parent = nullptr);

        auto setGstPipeline(std::string const& pipeline) -> void;

        [[nodiscard]] auto errorString() const -> QString;
        [[nodiscard]] auto error() const -> QMediaPlayer::Error;
        [[nodiscard]] auto isError() const -> bool;

        auto play() -> void;
        auto pause() -> void;
        auto stop() -> void;

        void setShowColorOverlay(bool show) { mShowColorOverlay = show; }
        void updateFrame(const QImage& frame);

    protected:
        void mouseMoveEvent(QMouseEvent* event) override;
        void enterEvent(QEvent* event) override;
        void leaveEvent(QEvent* event) override;
        void paintEvent(QPaintEvent* event) override;

    signals:
        void colorPicked(QPoint pos, QColor color);

    public:
        QImage getLastFrame() const { return mLastFrame; }
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
            QWidget* widget;
            QVBoxLayout* layout;

            QLabel* label;
            GstVideoWidget* gstVideoWidget;
        };

        QGridLayout* mMainLayout;
        std::unordered_map<std::string, GstVideoBox> mGstVideoBoxes;

        GstVideoGridWidget::Error mError;
        QString mErrorString;

        auto clearError() -> void;
        auto setError(Error error, std::string const& errorString) -> void;

    public:
        explicit GstVideoGridWidget(QWidget* parent = nullptr);

        auto addGstVideoWidget(std::string const& name, std::string const& pipeline) -> bool;
        auto getGstVideoWidget(std::string const& name) -> GstVideoWidget*;
        auto playVideo(std::string const& name) -> bool;
        auto pauseVideo(std::string const& name) -> bool;
        auto stopVideo(std::string const& name) -> bool;
        auto hideVideo(std::string const& name) -> bool;
        auto showVideo(std::string const& name) -> bool;

        [[nodiscard]] auto error() const -> GstVideoGridWidget::Error;
        [[nodiscard]] auto errorString() const -> QString;
        [[nodiscard]] auto isError() const -> bool;
    };
} // namespace mrover
