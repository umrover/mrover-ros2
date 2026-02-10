#include "ClickIkPanel.hpp"

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
        mClickLabel = new QLabel("Click: --", this);
        mClickLabel->setAlignment(Qt::AlignCenter);
        mInfoRow->addWidget(mToggleButton);
        mInfoRow->addWidget(mStatusLabel, 1);
        mInfoRow->addWidget(mClickLabel, 1);
        mInfoRow->addWidget(mResultLabel, 1);
        mInfoRow->addWidget(mFeedbackLabel, 1);
        mLayout->addLayout(mInfoRow);

        setLayout(mLayout);

        refreshStatus();
        refreshResultLabel(false);
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

    void ClickIkPanel::markRunning() {
        mRunning = true;
        refreshStatus();
    }

    void ClickIkPanel::updateFeedback(float distance) {
        mFeedbackLabel->setText(QString("Distance: %1").arg(static_cast<double>(distance), 0, 'f', 3));
    }

    void ClickIkPanel::updateClickPosition(std::uint32_t x, std::uint32_t y) {
        mClickLabel->setText(QString("Click: (%1, %2)").arg(x).arg(y));
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
