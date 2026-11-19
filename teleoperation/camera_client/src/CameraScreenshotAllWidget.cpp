#include "CameraScreenshotAllWidget.hpp"

using namespace mrover;

CameraScreenshotAllWidget::CameraScreenshotAllWidget(QWidget* parent) : QWidget(parent) {
    mMainLayout = new QVBoxLayout(this);

    // This appears to do absolutely nothing except add a blank space. This is the case in Config Widget too.
    // mFormWidget = new QWidget(this);
    // mFormLayout = new QFormLayout(mFormWidget);
    // mMainLayout->addWidget(mFormWidget);

    mSCAllButton = new QPushButton("Screenshot All", this);
    mMainLayout->addWidget(mSCAllButton);
    connect(mSCAllButton, &QPushButton::clicked, this, &CameraScreenshotAllWidget::onSCAllClicked);

    mMainLayout->addStretch();
    setLayout(mMainLayout);
}


void CameraScreenshotAllWidget::onSCAllClicked() {
    emit screenshotAllRequested();
}