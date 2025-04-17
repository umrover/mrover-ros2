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
    explicit GstVideoWidget(std::string const& pipeline, QWidget* parent = nullptr) : QVideoWidget(parent) {
        mPlayer = new QMediaPlayer(this);
        mPlayer->setVideoOutput(this);
        mPlayer->setMedia(QUrl(std::format("gst-pipeline: {} ! videoconvert ! xvimagesink name=\"qtvideosink\"", pipeline).c_str()));
        mPlayer->play();
    }

    explicit GstVideoWidget(QWidget* parent = nullptr) : GstVideoWidget("videotestsrc", parent) {}
};


class GstVideoBoxWidget : public QWidget {
    Q_OBJECT

    QVBoxLayout* layout;
    QLabel* label;
    GstVideoWidget* gstVideoWidget;

public:
    explicit GstVideoBoxWidget(std::string const& name, std::string const& pipeline, QWidget* parent = nullptr) : QWidget(parent) {
        layout = new QVBoxLayout(this);
        label = new QLabel(QString::fromStdString(name), this);
        gstVideoWidget = new GstVideoWidget(pipeline, this);
        gstVideoWidget->setMinimumSize(640, 360);

        layout->addWidget(label, 0);
        layout->addWidget(gstVideoWidget, 1);

        setLayout(layout);
    }
};


class GstVideoGridWidget : public QWidget {
    Q_OBJECT

    QGridLayout* mMainLayout;
    std::unordered_map<std::string, GstVideoBoxWidget*> mGstVideoBoxes;

public:
    explicit GstVideoGridWidget(QWidget* parent = nullptr) : QWidget(parent) {
        mMainLayout = new QGridLayout(this);
        mMainLayout->setContentsMargins(10, -1, 10, -1);
        mMainLayout->setSpacing(20);
        setLayout(mMainLayout);
    }

    auto addGstVideoWidget(std::string const& name, std::string const& pipeline) -> QWidget* {
        auto gstVideoBoxWidget = new GstVideoBoxWidget(name, pipeline, this);
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
};
