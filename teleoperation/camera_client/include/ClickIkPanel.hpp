#pragma once

#include "GstVideoWidgets.hpp"
#include "pch.hpp"

namespace mrover {

    class ClickIkPanel : public QWidget {
        Q_OBJECT

        QPushButton* mToggleButton;
        QHBoxLayout* mInfoRow;
        QLabel* mStatusLabel;
        QLabel* mFeedbackLabel;
        QLabel* mResultLabel;
        QVBoxLayout* mLayout;
        QWidget* mVideoContainer;
        QVBoxLayout* mVideoContainerLayout;
        mrover::GstVideoWidget* mVideoWidget;
        QPushButton* mSampleButton;
        QPushButton* mClearOverlayButton;
        QLabel* mSampleOverlay;
        QImage* mOverlayImage;


        bool mEnabled = false;
        bool mRunning = false;
        bool mShowOverlay = false;
        int mSuccessCount = 0;
        int mFailCount = 0;
        uint8_t const IMAGE_SAMPLE_RESOLUTION = 10;

    public:
        explicit ClickIkPanel(QWidget* parent = nullptr);

        auto placeZedWidget(GstVideoWidget* widget) -> void;
        [[nodiscard]] auto canSendClick() const -> bool { return mEnabled && !mRunning; }

    public slots:
        void updateFeedback(float distance);
        void updateResult(bool success);
        void enableOverlayWidget(mrover::action::IkImageSample::Result::SharedPtr const& imageSample);
        void markRunning();

    signals:
        void toggled(bool enabled);
        void sample();

    private:
        void onToggle();
        void onSample();
        void onClearOverlay();
        void refreshStatus();
        void refreshResultLabel(bool lastSuccess);
        void resizeEvent(QResizeEvent* event) override;
    };

} // namespace mrover
