#pragma once

#include "pch.hpp"

namespace mrover {

    class ClickIkPanel : public QWidget {
        Q_OBJECT

        QPushButton* mToggleButton;
        QHBoxLayout* mInfoRow;
        QLabel* mStatusLabel;
        QLabel* mFeedbackLabel;
        QLabel* mResultLabel;
        QLabel* mClickLabel;
        QVBoxLayout* mLayout;

        bool mEnabled = false;
        bool mRunning = false;
        int mSuccessCount = 0;
        int mFailCount = 0;

    public:
        explicit ClickIkPanel(QWidget* parent = nullptr);

        [[nodiscard]] auto canSendClick() const -> bool { return mEnabled && !mRunning; }

    public slots:
        void updateFeedback(float distance);
        void updateResult(bool success);
        void updateClickPosition(std::uint32_t x, std::uint32_t y);
        void markRunning();

    signals:
        void toggled(bool enabled);

    private:
        void onToggle();
        void refreshStatus();
        void refreshResultLabel(bool lastSuccess);
    };

} // namespace mrover
