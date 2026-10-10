#pragma once

#include "pch.hpp"
#include <string>

namespace mrover {

    namespace {
        constexpr auto DEFAULT_RTP_JITTER = std::chrono::milliseconds(500);
    }

    inline auto createRtpToRawSrc(std::uint16_t port, gst::video::Codec codec, std::chrono::milliseconds rtpJitter = DEFAULT_RTP_JITTER) -> std::string {
        std::string parser;
        if (codec == gst::video::Codec::H265) {
            parser = "! h265parse";
        } else if (codec == gst::video::Codec::H264) {
            parser = "! h264parse";
        }

        auto encoding = codec == gst::video::Codec::H265 ? "H265" : "H264";
        std::string caps = std::format(
            "application/x-rtp,media=video,encoding-name={},payload=96,clock-rate=90000",
            encoding);

        // return std::format("udpsrc port={} ! application/x-rtp,media=video ! rtpjitterbuffer latency={} ! {} {} ! decodebin", port, rtpJitter.count(), getRtpDepayloader(codec), parser);
        return std::format("udpsrc port={} ! {} ! rtpjitterbuffer latency={} ! {} {} ! decodebin", port, caps, rtpJitter.count(), getRtpDepayloader(codec), parser);
        // return R"(udpsrc port=8082 caps="application/x-rtp,media=video,encoding-name=H265,payload=96,clock-rate=90000" \ ! rtpjitterbuffer latency=500 \ ! rtph265depay ! h265parse ! decodebin \ ! videoconvert ! autovideosink)";
    }

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
        auto setWaiting(bool waiting) -> void;
    };
} // namespace mrover
