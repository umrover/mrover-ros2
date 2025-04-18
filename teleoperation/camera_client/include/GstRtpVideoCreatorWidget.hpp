#pragma once

#include "Gst.hpp"
#include "pch.hpp"

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
    explicit GstRtpVideoCreatorWidget(QWidget* parent = nullptr);

signals:
    void createRequested(std::string name, std::string pipeline);

public slots:
    void onCreateResult(bool success, QString const& errorMsg = {});

private slots:
    void onSubmitClicked();

private:
    void setWaiting(bool waiting);
};
