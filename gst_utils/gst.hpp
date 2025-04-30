#pragma once

#include <chrono>
#include <cstdint>
#include <format>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>

#include <gst/app/gstappsink.h>
#include <gst/app/gstappsrc.h>
#include <gst/gst.h>
#include <magic_enum.hpp>

namespace mrover::gst {

    // For props (should be convertible to string_view)
    template<typename... Args>
    concept AllConvertibleToStringView = (std::convertible_to<std::remove_cvref_t<Args>, std::string_view> && ...);

    inline auto addProperty(std::string_view key, auto value) -> std::string {
        if constexpr (std::is_same_v<decltype(value), bool>) {
            return std::format("{}={}", key, value ? "true" : "false");
        }

        return std::format("{}={}", key, value);
    }

    namespace video {
        enum class RawFormat : unsigned int {
            YUY2,
            NV12,
            I420,
        };

        constexpr auto toString(RawFormat format) -> std::string_view {
            if (auto name = magic_enum::enum_name(format); !name.empty()) {
                return name;
            }
            throw std::invalid_argument("Unsupported raw format");
        }

        constexpr auto getRawFormatFromStringView(std::string_view name) -> RawFormat {
            if (auto format = magic_enum::enum_cast<RawFormat>(name); format.has_value()) {
                return format.value();
            }
            throw std::invalid_argument("Unsupported raw format");
        }

        constexpr auto getMediaType(RawFormat) -> std::string_view {
            return "video/x-raw";
        }

        // TODO:(owen) I don't like the config-interval=1 for H265. Should be configurable somehow
#define CODEC_ITER(_F)                                                                                \
    _F(H264, "video/x-h264", "x264enc", "avdec_h264", "rtph264pay", "rtph264depay")                   \
    _F(H265, "video/x-h265", "x265enc", "avdec_h265", "rtph265pay config-interval=1", "rtph265depay") \
    _F(VP8, "video/x-vp8", "vp8enc", "vp8dec", "rtpvp8pay", "rtpvp8depay")                            \
    _F(VP9, "video/x-vp9", "vp9enc", "vp9dec", "rtpvp9pay", "rtpvp9depay")                            \
    _F(MPEG4, "video/mpeg4", "avenc_mpeg4", "avdec_mpeg4", "rtpmp4vpay", "rtpmp4vdepay")              \
    _F(JPEG, "image/jpeg", "jpegenc", "jpegdec", "rtpjpegpay", "rtpjpegdepay")

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

        constexpr auto toString(video::Codec codec) -> std::string_view {
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

#undef CODEC_ITER

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

        template<typename... Args>
        inline auto createDefaultEncoder(Codec codec, Args&&... extraProps) -> std::string
            requires AllConvertibleToStringView<Args...>
        {
            std::ostringstream oss;
            oss << getDefaultEncoder(codec);
            ((oss << ' ' << std::forward<Args>(extraProps)), ...);
            return oss.str();
        }

        template<typename... Args>
        inline auto createDefaultEncoder(std::string_view codec, Args&&... extraProps) -> std::string
            requires AllConvertibleToStringView<Args...>
        {
            return createDefaultEncoder(getCodecFromStringView(codec), std::forward<Args>(extraProps)...);
        }

        template<typename... Args>
        inline auto createDefaultDecoder(Codec codec, Args&&... extraProps) -> std::string
            requires AllConvertibleToStringView<Args...>
        {
            std::ostringstream oss;
            oss << getDefaultDecoder(codec);
            ((oss << ' ' << std::forward<Args>(extraProps)), ...);
            return oss.str();
        }

        template<typename... Args>
        inline auto createDefaultDecoder(std::string_view codec, Args&&... extraProps) -> std::string
            requires AllConvertibleToStringView<Args...>
        {
            return createDefaultDecoder(getCodecFromStringView(codec), std::forward<Args>(extraProps)...);
        }

        inline auto createRtpSink(std::string const& host, std::uint16_t port, video::Codec codec) {
            return std::format("{} ! udpsink host={} port={}", getRtpPayloader(codec), host, port);
        }

        constexpr auto DEFAULT_RTP_JITTER = std::chrono::milliseconds(500);

        inline auto createRtpToRawSrc(std::uint16_t port, video::Codec codec, std::chrono::milliseconds rtpJitter = DEFAULT_RTP_JITTER) -> std::string {
            return std::format("udpsrc port={} ! application/x-rtp,media=video ! rtpjitterbuffer latency={} ! {} ! decodebin", port, rtpJitter.count(), getRtpDepayloader(codec));
        }

    } // namespace video
} // namespace mrover::gst
