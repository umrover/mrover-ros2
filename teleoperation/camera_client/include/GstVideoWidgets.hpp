#pragma once

#include "pch.hpp"

namespace mrover {
    class GstVideoWidget : public QVideoWidget {
        Q_OBJECT

        QMediaPlayer* mPlayer;

    public:
        explicit GstVideoWidget(QWidget* parent = nullptr);

        auto setGstPipeline(std::string const& pipeline) -> void;

        [[nodiscard]] auto errorString() const -> QString;
        [[nodiscard]] auto error() const -> QMediaPlayer::Error;
        [[nodiscard]] auto isError() const -> bool;

        auto play() -> void;
        auto pause() -> void;
        auto stop() -> void;
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
        auto findVideoBox(std::string const& name) -> GstVideoBox*;

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
