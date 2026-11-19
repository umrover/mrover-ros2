#pragma once

#include "pch.hpp"


namespace mrover {
    class CameraScreenshotAllWidget : public QWidget {
        Q_OBJECT

        QVBoxLayout* mMainLayout;

        QWidget* mFormWidget;
        QFormLayout* mFormLayout;

        QPushButton* mSCAllButton;

    public:
        explicit CameraScreenshotAllWidget(QWidget* parent = nullptr);

    signals:
        void screenshotAllRequested();

    private slots:
        void onSCAllClicked();
    };
} // namespace mrover
