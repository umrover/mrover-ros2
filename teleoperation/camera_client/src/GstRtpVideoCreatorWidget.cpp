#include "GstRtpVideoCreatorWidget.hpp"

using namespace mrover;

inline auto fromStringView(std::string_view view) -> QString {
    return QString::fromUtf8(view.data(), static_cast<int>(view.size()));
}

GstRtpVideoCreatorWidget::GstRtpVideoCreatorWidget(QWidget* parent) : QWidget(parent) {
    mMainLayout = new QVBoxLayout(this);

    mFormWidget = new QWidget(this);
    mFormLayout = new QFormLayout(mFormWidget);
    mNameLineEdit = new QLineEdit(mFormWidget);
    mPortLineEdit = new QLineEdit(mFormWidget);
    mVideoCodecComboBox = new QComboBox(mFormWidget);
    for (auto const& codecName: gst::video::getAvailableCodecNames()) {
        mVideoCodecComboBox->addItem(fromStringView(codecName));
    }
    mFormLayout->addRow(tr("&Name"), mNameLineEdit);
    mFormLayout->addRow(tr("&Port*"), mPortLineEdit);
    mFormLayout->addRow(tr("&Codec*"), mVideoCodecComboBox);
    mMainLayout->addWidget(mFormWidget);

    mErrorLabel = new QLabel(this);
    mErrorLabel->setStyleSheet("QLabel { color : red; }");
    mErrorLabel->setVisible(false);
    mMainLayout->addWidget(mErrorLabel);

    mSubmitButton = new QPushButton(tr("Submit"), this);
    connect(mSubmitButton, &QPushButton::clicked, this, &GstRtpVideoCreatorWidget::onSubmitClicked);
    mMainLayout->addWidget(mSubmitButton);

    mMainLayout->addStretch();
    setLayout(mMainLayout);
}

void GstRtpVideoCreatorWidget::onCreateResult(bool success, QString const& errorMsg) {
    if (success) {
        mNameLineEdit->clear();
        mPortLineEdit->clear();
        mErrorLabel->setVisible(false);
    } else {
        mErrorLabel->setText(errorMsg);
        mErrorLabel->setVisible(true);
    }
    setWaiting(false);
}

auto GstRtpVideoCreatorWidget::setWaiting(bool waiting) -> void {
    mSubmitButton->setEnabled(!waiting);
    mNameLineEdit->setEnabled(!waiting);
    mPortLineEdit->setEnabled(!waiting);
    mVideoCodecComboBox->setEnabled(!waiting);
    mSubmitButton->setText(waiting ? tr("...") : tr("Submit"));
}

void GstRtpVideoCreatorWidget::onSubmitClicked() {
    bool ok;
    int port = mPortLineEdit->text().toInt(&ok);
    if (!ok) {
        mErrorLabel->setText(tr("Port must be a number"));
        mErrorLabel->setVisible(true);
        return;
    }
    if (port < 1024 || port > 65535) {
        mErrorLabel->setText(tr("Port must be between 1024 and 65535"));
        mErrorLabel->setVisible(true);
        return;
    }

    gst::video::Codec const codec = gst::video::getCodecFromStringView(mVideoCodecComboBox->currentText().toStdString());
    std::string const pipeline = createRtpToRawSrc(static_cast<std::uint16_t>(port), codec);

    std::string name;
    if (mNameLineEdit->text().isEmpty()) {
        name = "camera:" + std::to_string(port);
    } else {
        name = mNameLineEdit->text().toStdString();
    }

    setWaiting(true);
    emit createRequested(name, pipeline);
}
