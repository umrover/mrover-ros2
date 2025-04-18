#pragma once

#include "pch.hpp"

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

class GstVideoBoxWidget : public QWidget {
    Q_OBJECT

    QVBoxLayout* mLayout;
    QLabel* mLabel;
    GstVideoWidget* mGstVideoWidget;

public:
    explicit GstVideoBoxWidget(std::string const& name, QWidget* parent = nullptr);

    [[nodiscard]] auto getGstVideoWidget() -> GstVideoWidget*;
    [[nodiscard]] auto getLabel() -> QLabel*;

    auto setTitle(std::string const& title) -> void;
};

class GstVideoGridWidget : public QWidget {
    Q_OBJECT
public:
    enum Error {
        NoError,
        ExistsError,
        MediaPlayerError,
    };

private:
    QGridLayout* mMainLayout;
    std::unordered_map<std::string, GstVideoBoxWidget*> mGstVideoBoxes;

    GstVideoGridWidget::Error mError;
    QString mErrorString;

public:
    explicit GstVideoGridWidget(QWidget* parent = nullptr);

    auto addGstVideoWidget(std::string const& name, std::string const& pipeline) -> QWidget*;
    auto getGstVideoWidget(std::string const& name) -> QWidget*;
    auto hideGstVideoWidget(std::string const& name) -> bool;
    auto showGstVideoWidget(std::string const& name) -> bool;

    [[nodiscard]] auto error() const -> GstVideoGridWidget::Error;
    [[nodiscard]] auto errorString() const -> QString;
    [[nodiscard]] auto isError() const -> bool;
};
