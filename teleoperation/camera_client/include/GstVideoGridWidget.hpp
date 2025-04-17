#pragma once

#include <format>
#include <string>
#include <unordered_map>

#include <QCheckBox>
#include <QGridLayout>
#include <QLabel>
#include <QMediaPlayer>
#include <QVBoxLayout>
#include <QVideoWidget>
#include <QWidget>

class GstVideoWidget : public QVideoWidget {
    Q_OBJECT

    QMediaPlayer* mPlayer;

public:
    explicit GstVideoWidget(QWidget* parent = nullptr) : QVideoWidget(parent) {
        mPlayer = new QMediaPlayer(this);
        mPlayer->setVideoOutput(this);
    }

    auto setGstPipeline(std::string const& pipeline) -> void {
        mPlayer->setMedia(QUrl(std::format("gst-pipeline: {} ! videoconvert ! xvimagesink name=\"qtvideosink\"", pipeline).c_str()));
        play();
    }

    [[nodiscard]] auto errorString() const -> QString {
        return mPlayer->errorString();
    }

    [[nodiscard]] auto error() const -> QMediaPlayer::Error {
        return mPlayer->error();
    }

    [[nodiscard]] auto isError() const -> bool {
        return mPlayer->error() != QMediaPlayer::NoError;
    }

    auto play() -> void {
        mPlayer->play();
    }

    auto pause() -> void {
        mPlayer->pause();
    }

    auto stop() -> void {
        mPlayer->stop();
    }
};


/** 
 * @class GstVideoBoxWidget
 * @brief A simple widget that acts as a wrapper class to format a QLabel and a GstVideoWidget.
 *
 * It is used to display a video stream with a title.
 */
class GstVideoBoxWidget : public QWidget {
    Q_OBJECT

    QVBoxLayout* mLayout;
    QLabel* mLabel;
    GstVideoWidget* mGstVideoWidget;

public:
    explicit GstVideoBoxWidget(std::string const& name, QWidget* parent = nullptr) : QWidget(parent) {
        mLayout = new QVBoxLayout(this);
        mLabel = new QLabel(QString::fromStdString(name), this);
        mGstVideoWidget = new GstVideoWidget(this);
        mGstVideoWidget->setMinimumSize(640, 360);

        mLayout->addWidget(mLabel, 0);
        mLayout->addWidget(mGstVideoWidget, 1);

        setLayout(mLayout);
    }

    [[nodiscard]] auto getGstVideoWidget() -> GstVideoWidget* {
        return mGstVideoWidget;
    }

    [[nodiscard]] auto getLabel() -> QLabel* {
        return mLabel;
    }

    auto setTitle(std::string const& title) -> void {
        mLabel->setText(QString::fromStdString(title));
    }
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
    explicit GstVideoGridWidget(QWidget* parent = nullptr) : QWidget(parent), mError(NoError) {
        mMainLayout = new QGridLayout(this);
        mMainLayout->setContentsMargins(10, -1, 10, -1);
        mMainLayout->setSpacing(20);
        setLayout(mMainLayout);
    }

    auto addGstVideoWidget(std::string const& name, std::string const& pipeline) -> QWidget* {
        if (mGstVideoBoxes.find(name) != mGstVideoBoxes.end()) {
            mError = ExistsError;
            mErrorString = "Camera name already exists";
            return nullptr;
        }

        auto gstVideoBoxWidget = new GstVideoBoxWidget(name, this);
        GstVideoWidget* gstVideoWidget = gstVideoBoxWidget->getGstVideoWidget();
        gstVideoWidget->setGstPipeline(pipeline);
        if (gstVideoWidget->isError()) {
            mError = MediaPlayerError;
            mErrorString = gstVideoWidget->errorString();
            delete gstVideoBoxWidget;
            return nullptr;
        }
        gstVideoBoxWidget->setFixedSize(640, 360);

        int const index = static_cast<int>(mGstVideoBoxes.size());
        mMainLayout->addWidget(gstVideoBoxWidget, index / 2, index % 2, Qt::AlignCenter);

        mGstVideoBoxes.emplace(name, gstVideoBoxWidget);

        return gstVideoBoxWidget;
    }

    auto getGstVideoWidget(std::string const& name) -> QWidget* {
        if (mGstVideoBoxes.find(name) == mGstVideoBoxes.end()) {
            return nullptr;
        }
        return mGstVideoBoxes.at(name);
    }

    auto hideGstVideoWidget(std::string const& name) -> bool {
        if (mGstVideoBoxes.find(name) == mGstVideoBoxes.end()) {
            return false;
        }
        mGstVideoBoxes.at(name)->setVisible(false);
        return true;
    }

    auto showGstVideoWidget(std::string const& name) -> bool {
        if (mGstVideoBoxes.find(name) == mGstVideoBoxes.end()) {
            return false;
        }
        mGstVideoBoxes.at(name)->setVisible(true);
        return true;
    }

    [[nodiscard]] auto error() const -> GstVideoGridWidget::Error {
        return mError;
    }

    [[nodiscard]] auto errorString() const -> QString {
        return mErrorString;
    }

    [[nodiscard]] auto isError() const -> bool {
        return mError != NoError;
    }
};
