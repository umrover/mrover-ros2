#pragma once

#include "pch.hpp"
#include <string>

namespace mrover {

    namespace {
        constexpr auto DEFAULT_RTP_JITTER = std::chrono::milliseconds(500);
    }

    inline auto createRtpToRawSrc(std::uint16_t port, gst::video::Codec codec, std::chrono::milliseconds rtpJitter = DEFAULT_RTP_JITTER) -> std::string {
        std::string parser;
        std::string encoding;

        switch(codec){
            case gst::video::Codec::H265:
                parser = "! h265parse";
                encoding = "H265";
            break;
            case gst::video::Codec::H264:
                parser = "! h264parse";
                encoding = "H264";
            break;
            case gst::video::Codec::AV1 :
                parser = "! av1parse";
                encoding = "AV1";
            break;
            case gst::video::Codec::JPEG:
                parser = "! jpegparse";
                encoding = "JPEG";
            break;
            case gst::video::Codec::MPEG4:
                parser = "! mpeg4videoparse";
                encoding = "MPEG4";
            break;
            case gst::video::Codec::VP8 :
                // TODO VP8 doesn't use a parser
                encoding = "VP8";
            break;
            case gst::video::Codec::VP9:
                parser = "! vp9parse";
                encoding = "VP9";
            break;
            
            default:

        }
        
        std::string caps = std::format("application/x-rtp,media=video,encoding-name={},payload=96,clock-rate=90000", encoding);

        return std::format("udpsrc port={} ! {} ! rtpjitterbuffer latency={} ! {} {} ! decodebin", port, caps, rtpJitter.count(), getRtpDepayloader(codec), parser);
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
