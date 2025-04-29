#pragma once

#include <stdexcept>
#include <string_view>

#include <magic_enum.hpp>

namespace mrover::gst::Video {

    // TODO:(owen) I don't like the config-interval=1 for H265. Should be configurable somehow
#define CODEC_ITER(_F)                                                                                \
    _F(H264, "video/x-h264", "x264enc", "avdec_h264", "rtph264pay", "rtph264depay")                   \
    _F(H265, "video/x-h265", "x265enc", "avdec_h265", "rtph265pay", "rtph265depay config-interval=1") \
    _F(VP8, "video/x-vp8", "vp8enc", "vp8dec", "rtpvp8pay", "rtpvp8depay")                            \
    _F(VP9, "video/x-vp9", "vp9enc", "vp9dec", "rtpvp9pay", "rtpvp9depay")                            \
    _F(MPEG4, "video/mpeg4", "avenc_mpeg4", "avdec_mpeg4", "rtpmpeg4pay", "rtpmpeg4depay")            \
    _F(MJPEG, "image/jpeg", "jpegenc", "jpegdec", "rtpjpegpay", "rtpjpegdepay")

    enum class Codec : unsigned int {
#define F(name, ...) name,
        CODEC_ITER(F)
#undef F
    };

    struct CodecInfo {
        Codec codec;
        std::string_view mediaType;
        std::string_view defaultEncoder;
        std::string_view defaultDecoder;
        std::string_view rtpPayloader;
        std::string_view rtpDepayloader;
    };

    constexpr auto toString(Video::Codec codec) -> std::string_view {
        if (auto name = magic_enum::enum_name(codec); !name.empty()) {
            return name;
        }
        throw std::invalid_argument("Unsupported codec");
    }

    constexpr auto getCodecFromStringView(std::string_view name) -> Codec {
        if (auto codec = magic_enum::enum_cast<Codec>(name); codec.has_value()) {
            return codec.value();
        }
        throw std::invalid_argument("Unsupported codec");
    }
    constexpr auto getCodecFromString(std::string const& name) -> Codec {
        return getCodecFromStringView(name);
    }

    constexpr auto getAvailableCodecs() -> std::array<Codec, magic_enum::enum_count<Codec>()> {
        return magic_enum::enum_values<Codec>();
    }

    constexpr auto getAvailableCodecNames() -> std::array<std::string_view, magic_enum::enum_count<Codec>()> {
        return magic_enum::enum_names<Codec>();
    }

    template<typename Getter>
    constexpr auto getCodecInfo(Codec codec, Getter getter) -> decltype(getter(CodecInfo{})) {
        switch (codec) {
#define F(name, mt, enc, dec, rtp_pay, rtp_depay) \
    case Codec::name:                             \
        return getter(CodecInfo{Codec::name, mt, enc, dec, rtp_pay, rtp_depay});

            CODEC_ITER(F)
#undef F
            default:
                throw std::invalid_argument("Unsupported codec");
        }
    }

    constexpr auto getMediaType(Codec codec) -> std::string_view {
        return getCodecInfo(codec, [](CodecInfo const& info) { return info.mediaType; });
    }

    constexpr auto getDefaultEncoder(Codec codec) -> std::string_view {
        return getCodecInfo(codec, [](CodecInfo const& info) { return info.defaultEncoder; });
    }

    constexpr auto getDefaultDecoder(Codec codec) -> std::string_view {
        return getCodecInfo(codec, [](CodecInfo const& info) { return info.defaultDecoder; });
    }

    constexpr auto getRtpPayloader(Codec codec) -> std::string_view {
        return getCodecInfo(codec, [](CodecInfo const& info) { return info.rtpPayloader; });
    }

    constexpr auto getRtpDepayloader(Codec codec) -> std::string_view {
        return getCodecInfo(codec, [](CodecInfo const& info) { return info.rtpDepayloader; });
    }

#undef COEC_ITER

} // namespace mrover::gst::Video
