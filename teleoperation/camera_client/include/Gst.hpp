#pragma once

#include <chrono>
#include <cstdint>
#include <format>
#include <string>
#include <vector>

namespace Gst {

#define VIDEO_CODECS_ITER(_F)          \
    _F(H264, "H264", 0, __VA_ARGS__)   \
    _F(H265, "H265", 1, __VA_ARGS__)   \
    _F(VP8, "VP8", 2, __VA_ARGS__)     \
    _F(VP9, "VP9", 3, __VA_ARGS__)     \
    _F(MPEG1, "MPEG1", 4, __VA_ARGS__) \
    _F(MPEG2, "MPEG2", 5, __VA_ARGS__) \
    _F(MPEG4, "MPEG4", 6, __VA_ARGS__) \
    _F(MJPEG, "MJPEG", 7, __VA_ARGS__)

    enum VideoCodec : uint {
#define DECL_ENUMS(uc, str, i, ...) uc = (i),
        VIDEO_CODECS_ITER(DECL_ENUMS)
#undef DECL_ENUMS
    };

    inline auto getAvailableCodecs() -> std::vector<VideoCodec> {
        return {
#define DECL_ENUMS_VECTOR(uc, str, i, ...) VideoCodec::uc,
                VIDEO_CODECS_ITER(DECL_ENUMS_VECTOR)};
    }

    inline auto getVideoCodecString(VideoCodec codec) -> std::string {
#define DECL_STRINGS(uc, str, i, ...) \
    case VideoCodec::uc: {            \
        return str;                   \
    }
        switch (codec) {
            VIDEO_CODECS_ITER(DECL_STRINGS)
            default: {
                throw std::invalid_argument("Unsupported codec");
            }
        }
#undef DECL_STRINGS
    }

    inline auto getVideoCodecFromString(std::string const& codec) -> VideoCodec {
#define DECL_CODEC_FROM_STRINGS(uc, str, i, ...) \
    if (codec == str) {                          \
        return VideoCodec::uc;                   \
    }
        VIDEO_CODECS_ITER(DECL_CODEC_FROM_STRINGS)
#undef DECL_CODEC_FROM_STRINGS

        throw std::invalid_argument("Unsupported codec");
    }

    namespace PipelineStrings {
        constexpr auto DEFAULT_RTP_JITTER = std::chrono::milliseconds(200);

        inline auto createRtpToRawString(std::uint16_t port, VideoCodec codec, std::chrono::milliseconds rtpJitter = DEFAULT_RTP_JITTER) -> std::string {
            char const* depay = nullptr;
            char const* decoder = nullptr;

            switch (codec) {
                case VideoCodec::H264: {
                    depay = "rtph264depay";
                    decoder = "avdec_h264";
                    break;
                }
                case VideoCodec::H265: {
                    depay = "rtph265depay";
                    decoder = "avdec_h265";
                    break;
                }
                case VideoCodec::VP8: {
                    depay = "rtpvp8depay";
                    decoder = "avdec_vp8";
                    break;
                }
                case VideoCodec::VP9: {
                    depay = "rtpvp9depay";
                    decoder = "avdec_vp9";
                    break;
                }
                case VideoCodec::MPEG1: {
                    depay = "rtpmp1sdepay";
                    decoder = "avdec_mpegvideo";
                    break;
                }
                case VideoCodec::MPEG2: {
                    depay = "rtpmp2tdepay";
                    decoder = "avdec_mpeg2video";
                    break;
                }
                case VideoCodec::MPEG4: {
                    depay = "rtpmp4vdepay";
                    decoder = "avdec_mpeg4";
                    break;
                }
                case VideoCodec::MJPEG: {
                    depay = "rtpjpegdepay";
                    decoder = "avdec_mjpeg";
                    break;
                }
                default: {
                    throw std::invalid_argument("Unsupported codec");
                }
            }

            return std::format("udpsrc port={} ! application/x-rtp,media=video ! rtpjitterbuffer latency={} ! {} ! {}", port, rtpJitter.count(), depay, decoder);
        }
    } // namespace PipelineStrings

}; // namespace Gst
