#pragma once

#include "pch.hpp"


namespace mrover {
    class CameraClientNode;

    class CameraConfigWidget : public QWidget {
        Q_OBJECT

        QVBoxLayout* mMainLayout;

        QWidget* mFormWidget;
        QFormLayout* mFormLayout;

        QPushButton* mArmButton;
        QPushButton* mScienceButton;
        QPushButton* mAutonButton;

    public:
        explicit CameraConfigWidget(QWidget* parent = nullptr);

    signals:
        void loadCameraConfigSignal(std::string const& config);

    private slots:
        void onArmClicked();
        void onScienceClicked();
        void onAutonClicked();
    };
} // namespace mrover
