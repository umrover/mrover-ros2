#include "CameraConfigWidget.hpp"
#include "CameraClientNode.hpp"

using namespace mrover;

CameraConfigWidget::CameraConfigWidget(QWidget* parent) : QWidget(parent) {
    mMainLayout = new QVBoxLayout(this);

    mFormWidget = new QWidget(this);
    mFormLayout = new QFormLayout(mFormWidget);
    mMainLayout->addWidget(mFormWidget);

    mArmButton = new QPushButton(tr(CAMERA_CONFIGS[CAMERA_CONFIG::ARM]), this);
    mScienceButton = new QPushButton(tr(CAMERA_CONFIGS[CAMERA_CONFIG::SCIENCE]), this);
    mAutonButton = new QPushButton(tr(CAMERA_CONFIGS[CAMERA_CONFIG::AUTON]), this);

    mMainLayout->addWidget(mArmButton);
    mMainLayout->addWidget(mScienceButton);
    mMainLayout->addWidget(mAutonButton);

    connect(mArmButton, &QPushButton::clicked, this, &CameraConfigWidget::onArmClicked);
    connect(mScienceButton, &QPushButton::clicked, this, &CameraConfigWidget::onScienceClicked);
    connect(mAutonButton, &QPushButton::clicked, this, &CameraConfigWidget::onAutonClicked);

    mMainLayout->addStretch();
    setLayout(mMainLayout);
}

void CameraConfigWidget::onArmClicked() {
    emit loadCameraConfigSignal(CAMERA_CONFIGS[CAMERA_CONFIG::ARM]);
}

void CameraConfigWidget::onScienceClicked() {
    emit loadCameraConfigSignal(CAMERA_CONFIGS[CAMERA_CONFIG::SCIENCE]);
}

void CameraConfigWidget::onAutonClicked() {
    emit loadCameraConfigSignal(CAMERA_CONFIGS[CAMERA_CONFIG::AUTON]);
}
