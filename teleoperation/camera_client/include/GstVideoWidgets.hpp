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

    class GstVideoWidget : public QWidget {
        Q_OBJECT

        GstElement* mPipeline = nullptr;
        std::string mPipelineString;
        bool mStarted = false;
        bool mIsError = false;
        QString mErrorString;

    public:
        explicit GstVideoWidget(QWidget* parent = nullptr);
        ~GstVideoWidget() override;

        auto setGstPipeline(std::string const& pipeline) -> void;

        [[nodiscard]] auto errorString() const -> QString;
        [[nodiscard]] auto isError() const -> bool;

        auto play() -> void;
        auto pause() -> void;
        auto stop() -> void;

    protected:
        void showEvent(QShowEvent* event) override;

    private:
        void startPipeline();
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

        [[nodiscard]] auto error() const -> GstVideoGridWidget::Error;
        [[nodiscard]] auto errorString() const -> QString;
        [[nodiscard]] auto isError() const -> bool;
    };

} // namespace mrover
