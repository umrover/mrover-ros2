#include "ClickIkPanel.hpp"
#include "mrover/action/detail/ik_image_sample__struct.hpp"
#include <qnamespace.h>
#include <qpixmap.h>

namespace mrover {

    ClickIkPanel::ClickIkPanel(QWidget* parent) : QWidget(parent) {
        mLayout = new QVBoxLayout(this);

        mInfoRow = new QHBoxLayout();
        mToggleButton = new QPushButton("Enable ClickIK", this);
        mToggleButton->setCheckable(true);
        connect(mToggleButton, &QPushButton::clicked, this, &ClickIkPanel::onToggle);
        mStatusLabel = new QLabel(this);
        mStatusLabel->setAlignment(Qt::AlignCenter);
        mResultLabel = new QLabel(this);
        mResultLabel->setAlignment(Qt::AlignCenter);
        mFeedbackLabel = new QLabel(this);
        mFeedbackLabel->setAlignment(Qt::AlignCenter);
        mSampleButton = new QPushButton("Sample", this);
        mSampleButton->setCheckable(true);
        connect(mSampleButton, &QPushButton::clicked, this, &ClickIkPanel::onSample);
        mClearOverlayButton = new QPushButton("Clear Overlay", this);
        mClearOverlayButton->setCheckable(true);
        connect(mClearOverlayButton, &QPushButton::clicked, this, &ClickIkPanel::onClearOverlay);
        mInfoRow->addWidget(mToggleButton);
        mInfoRow->addWidget(mStatusLabel, 1);
        mInfoRow->addWidget(mResultLabel, 1);
        mInfoRow->addWidget(mFeedbackLabel, 1);
        mInfoRow->addWidget(mSampleButton, 1);
        mInfoRow->addWidget(mClearOverlayButton, 1);
        mLayout->addLayout(mInfoRow);

        mVideoContainer = new QWidget(this);
        mVideoContainerLayout = new QVBoxLayout(mVideoContainer);
        mVideoContainerLayout->setContentsMargins(0, 0, 0, 0);
        mVideoContainer->setLayout(mVideoContainerLayout);
        mLayout->addWidget(mVideoContainer, 1);

        mSampleOverlay = new QLabel(this);
        mSampleOverlay->setAttribute(Qt::WA_TransparentForMouseEvents);
        mSampleOverlay->setStyleSheet("background: transparent;");
        mSampleOverlay->setScaledContents(true);

        mOverlayImage = new QImage(1280, 720, QImage::Format_ARGB32_Premultiplied);
        mSampleOverlay->setPixmap(QPixmap::fromImage(*mOverlayImage));
        mSampleOverlay->setVisible(false);

        setLayout(mLayout);
        setMinimumWidth(640);

        refreshStatus();
        refreshResultLabel(false);
    }

    auto ClickIkPanel::placeZedWidget(GstVideoWidget* widget) -> void {
        mVideoWidget = widget;
        mVideoWidget->setParent(mVideoContainer);
        mVideoWidget->setGeometry(mVideoContainer->rect());
        mSampleOverlay->setParent(mVideoContainer);
        mSampleOverlay->setGeometry(mVideoWidget->rect());
        mSampleOverlay->raise();
        mVideoWidget->show();
    }

    void ClickIkPanel::resizeEvent(QResizeEvent* event) {
        QWidget::resizeEvent(event); // Let the base class handle layout first

        if (mVideoWidget && mVideoContainer) {
            // Manually snap the video widget to the container's exact dimensions
            mVideoWidget->setGeometry(mVideoContainer->rect());
            mSampleOverlay->setGeometry(mVideoWidget->rect());

            // Synchronize the internal coordinate system to this new size
            mVideoWidget->setImageSize(mVideoWidget->width(), mVideoWidget->height());
            mSampleOverlay->setPixmap(QPixmap::fromImage(*mOverlayImage));
        }
    }

    void ClickIkPanel::onToggle() {
        mEnabled = mToggleButton->isChecked();
        mToggleButton->setText(mEnabled ? "Disable ClickIK" : "Enable ClickIK");
        if (!mEnabled) {
            mSuccessCount = 0;
            mFailCount = 0;
            mRunning = false;
        }
        refreshStatus();
        refreshResultLabel(false);
        emit toggled(mEnabled);
    }

    void ClickIkPanel::onSample() {
        emit sample();
    };

    void ClickIkPanel::onClearOverlay() {
        mSampleOverlay->setVisible(false);
    };

    void ClickIkPanel::enableOverlayWidget(mrover::action::IkImageSample::Result::SharedPtr imageSample) {
        if (imageSample) {
            QColor green(0, 255, 0, 80);
            QColor red(255, 0, 0, 80);

            int resX = 1280;
            int resY = 720;

            int gridW = 128;
            int gridH = 72;
            int expected = gridW * gridH;

            if (static_cast<int>(imageSample->success.size()) < expected) {
                // bail / warn
                return;
            }

            int cellW = resX / gridW; // should be 10
            int cellH = resY / gridH; // should be 10

            for (int y = 0; y < gridH; ++y) {
                for (int x = 0; x < gridW; ++x) {
                    int idx = y * gridW + x; // row-major
                    QColor cellColor = imageSample->success[idx] || (x == 0 && y == 0) || (x == 127 && y == 71) ? green : red;

                    int startX = x * cellW;
                    int startY = y * cellH;

                    for (int py = 0; py < cellH; ++py) {
                        for (int px = 0; px < cellW; ++px) {
                            mOverlayImage->setPixelColor(startX + px, startY + py, cellColor);
                        }
                    }
                }
            }
        }
        mSampleOverlay->setPixmap(QPixmap::fromImage(*mOverlayImage));
        mSampleOverlay->setVisible(true);
    }

    void ClickIkPanel::markRunning() {
        mRunning = true;
        refreshStatus();
    }

    void ClickIkPanel::updateFeedback(float distance) {
        mFeedbackLabel->setText(QString("Distance: %1").arg(static_cast<double>(distance), 0, 'f', 3));
    }

    void ClickIkPanel::updateResult(bool success) {
        mRunning = false;
        if (success) {
            ++mSuccessCount;
        } else {
            ++mFailCount;
        }
        refreshStatus();
        refreshResultLabel(success);
    }

    void ClickIkPanel::refreshStatus() {
        if (!mEnabled) {
            mStatusLabel->setText("Status: OFF");
            mFeedbackLabel->setText("Distance: --");
        } else if (mRunning) {
            mStatusLabel->setText("Status: WAITING");
        } else {
            mStatusLabel->setText("Status: READY");
        }
    }

    void ClickIkPanel::refreshResultLabel(bool lastSuccess) {
        if (mSuccessCount == 0 && mFailCount == 0) {
            mResultLabel->setText("Last: --  |  OK: 0  |  Failed: 0");
        } else {
            QString last = lastSuccess ? "Last: OK" : "Last: FAILED";
            mResultLabel->setText(QString("%1  |  OK: %2  |  Failed: %3").arg(last).arg(mSuccessCount).arg(mFailCount));
        }
    }

} // namespace mrover
