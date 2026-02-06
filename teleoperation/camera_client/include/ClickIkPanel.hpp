#pragma once

#include "pch.hpp"
#include "GstVideoWidgets.hpp"

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

        bool mEnabled = false;
        bool mRunning = false;
        int mSuccessCount = 0;
        int mFailCount = 0;

    public:
        explicit ClickIkPanel(QWidget* parent = nullptr);

        auto placeZedWidget(GstVideoWidget* widget) -> void;
        [[nodiscard]] auto canSendClick() const -> bool { return mEnabled && !mRunning; }

    public slots:
        void updateFeedback(float distance);
        void updateResult(bool success);
        void markRunning();

    signals:
        void toggled(bool enabled);

    private:
        void onToggle();
        void refreshStatus();
        void refreshResultLabel(bool lastSuccess);
    };

} // namespace mrover
