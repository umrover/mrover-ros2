#pragma once

#include <QComboBox>
#include <QDockWidget>
#include <QFormLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>

#include "Gst.hpp"

/**
 * @class GstRtpVideoCreatorWidget
 * @brief Widget to create a new RTP video source
 *
 * This widget allows the user to create a new RTP video source by specifying the name, port, and codec.
 * It emits a createRequested signal when the user clicks the submit button, which provides the name they
 * assigned the video source and the gstreamer pipeline which generally follows the structure (udpsrc --> rtpjitterbuffer --> rtpdepay --> decoder).
 */
class GstRtpVideoCreatorWidget : public QWidget {
    Q_OBJECT

    QVBoxLayout* mMainLayout;

    QWidget* mFormWidget;
    QFormLayout* mFormLayout;
    QLineEdit* mNameLineEdit;
    QLineEdit* mPortLineEdit;
    QComboBox* mVideoCodecComboBox;

    QPushButton* mSubmitButton;

    QLabel* mErrorLabel;

public:
    explicit GstRtpVideoCreatorWidget(QWidget* parent = nullptr) : QWidget(parent) {
        mMainLayout = new QVBoxLayout(this);

        // Form
        mFormWidget = new QWidget(this);
        mFormLayout = new QFormLayout(mFormWidget);
        mNameLineEdit = new QLineEdit(mFormWidget);
        mPortLineEdit = new QLineEdit(mFormWidget);
        mVideoCodecComboBox = new QComboBox(mFormWidget);
        for (auto const& codec: Gst::getAvailableCodecs()) {
            mVideoCodecComboBox->addItem(QString::fromStdString(Gst::getVideoCodecString(codec)));
        }
        mFormLayout->addRow(tr("&Name"), mNameLineEdit);
        mFormLayout->addRow(tr("&Port*"), mPortLineEdit);
        mFormLayout->addRow(tr("&Codec*"), mVideoCodecComboBox);
        mMainLayout->addWidget(mFormWidget);

        // Error
        mErrorLabel = new QLabel(this);
        mErrorLabel->setStyleSheet("QLabel { color : red; }");
        mErrorLabel->setVisible(false);
        mMainLayout->addWidget(mErrorLabel);

        // Submit
        mSubmitButton = new QPushButton(tr("Submit"), this);
        connect(mSubmitButton, &QPushButton::clicked, this, &GstRtpVideoCreatorWidget::onSubmitClicked);
        mMainLayout->addWidget(mSubmitButton);

        mMainLayout->addStretch();

        setLayout(mMainLayout);
    }

signals:
    void createRequested(std::string name, std::string pipeline);

public slots:
    void onCreateResult(bool success, QString const& errorMsg = {}) {
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

private:
    void setWaiting(bool waiting) {
        mSubmitButton->setEnabled(!waiting);
        mNameLineEdit->setEnabled(!waiting);
        mPortLineEdit->setEnabled(!waiting);
        mVideoCodecComboBox->setEnabled(!waiting);
        mSubmitButton->setText(waiting ? tr("...") : tr("Submit"));
    }

private slots:
    void onSubmitClicked() {
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

        Gst::VideoCodec const codec = Gst::getVideoCodecFromString(mVideoCodecComboBox->currentText().toStdString());
        std::string const pipeline = Gst::PipelineStrings::createRtpToRawString(static_cast<std::uint16_t>(port), codec);

        if (mNameLineEdit->text().isEmpty()) {
            mErrorLabel->setText(tr("Name cannot be empty"));
            mErrorLabel->setVisible(true);
            return;
        }

        setWaiting(true);

        emit createRequested(mNameLineEdit->text().toStdString(), pipeline);
    }
};
