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

    class PipelineBuilder {
        std::vector<std::string> mElements;

    public:
        PipelineBuilder() = default;

        auto pushBack(std::string_view elem) -> PipelineBuilder& {
            mElements.emplace_back(elem);
            return *this;
        }

        template<typename... Args>
        auto pushBack(std::string_view elem, Args&&... props) -> PipelineBuilder&
            requires AllConvertibleToStringView<Args...>
        {
            std::ostringstream oss;
            oss << elem;
            ((oss << ' ' << std::forward<Args>(props)), ...);
            mElements.emplace_back(oss.str());
            return *this;
        }

        auto popBack() -> PipelineBuilder& {
            if (!mElements.empty()) {
                mElements.pop_back();
            }
            return *this;
        }

        auto insert(std::size_t index, std::string_view elem) -> PipelineBuilder& {
            if (index <= mElements.size()) {
                mElements.insert(mElements.begin() + static_cast<std::ptrdiff_t>(index), std::string(elem));
            }
            return *this;
        }

        template<typename... Args>
        auto insert(std::size_t index, std::string_view elem, Args&&... props) -> PipelineBuilder&
            requires AllConvertibleToStringView<Args...>
        {
            if (index <= mElements.size()) {
                std::ostringstream oss;
                oss << elem;
                ((oss << ' ' << std::forward<Args>(props)), ...);
                mElements.insert(mElements.begin() + static_cast<std::ptrdiff_t>(index), oss.str());
            }
            return *this;
        }

        auto remove(std::size_t index) -> PipelineBuilder& {
            if (index < mElements.size()) {
                mElements.erase(mElements.begin() + static_cast<std::ptrdiff_t>(index));
            }
            return *this;
        }

        auto clear() -> PipelineBuilder& {
            mElements.clear();
            return *this;
        }

        [[nodiscard]] auto size() const -> std::size_t {
            return mElements.size();
        }

        template<typename... Args>
        auto addPropsToElement(std::size_t index, Args&&... props) -> PipelineBuilder&
            requires AllConvertibleToStringView<Args...>
        {
            if (index < mElements.size()) {
                std::ostringstream oss;
                oss << mElements[index];
                ((oss << ' ' << std::forward<Args>(props)), ...);
                mElements[index] = oss.str();
            }
            return *this;
        }

        [[nodiscard]] auto str() const -> std::string {
            std::ostringstream oss;
            for (std::size_t i = 0; i < mElements.size(); ++i) {
                oss << mElements[i];
                if (i != mElements.size() - 1) {
                    oss << " ! ";
                }
            }
            return oss.str();
        }
    };

    class PipelineWrapper {
        std::string mLaunch{};
        GstElement* mPipeline{nullptr};
        bool mIsPlaying{false};

    public:
        PipelineWrapper() = default;

        explicit PipelineWrapper(std::string launch)
            : mLaunch(std::move(launch)) {
        }

        ~PipelineWrapper() {
            if (mPipeline) {
                gst_element_set_state(mPipeline, GST_STATE_NULL);
                gst_object_unref(mPipeline);
            }
        }

        auto init() -> void {
            mPipeline = gst_parse_launch(mLaunch.c_str(), nullptr);
            if (!mPipeline) {
                throw std::runtime_error{"Failed to create pipeline"};
            }
        }

        auto stop() -> void {
            if (gst_element_set_state(mPipeline, GST_STATE_NULL) == GST_STATE_CHANGE_FAILURE) {
                throw std::runtime_error{"gst_element_set_state to GST_STATE_NULL failed"};
            }
            mIsPlaying = false;
        }

        auto pause() -> void {
            if (gst_element_set_state(mPipeline, GST_STATE_PAUSED) == GST_STATE_CHANGE_FAILURE) {
                throw std::runtime_error{"gst_element_set_state to GST_STATE_PAUSED failed"};
            }
            mIsPlaying = false;
        }

        auto play() -> void {
            if (gst_element_set_state(mPipeline, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
                throw std::runtime_error{"gst_element_set_state to GST_STATE_PLAYING failed"};
            }
            mIsPlaying = true;
        }

        [[nodiscard]] auto getLaunchString() const -> std::string {
            return mLaunch;
        }

        [[nodiscard]] auto getPipelineElement() const -> GstElement* {
            return mPipeline;
        }

        [[nodiscard]] auto isPlaying() const -> bool {
            return mIsPlaying;
        }

        auto addBusWatcher(GstBusFunc func, gpointer user_data) -> guint {
            if (!mPipeline) {
                throw std::runtime_error{"Pipeline not initialized"};
            }
            GstBus* mBus = gst_element_get_bus(mPipeline);
            if (!mBus) {
                throw std::runtime_error{"Failed to get bus from pipeline"};
            }
            guint result = gst_bus_add_watch(mBus, func, user_data);

            gst_object_unref(mBus);

            return result;
        }
    };

    namespace video {
        enum class RawFormat : unsigned int {
            YUY2,
            NV12,
            I420,
            BGRA,
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
            return std::format("{} ! queue ! udpsink host={} port={}", getRtpPayloader(codec), host, port);
        }

        constexpr auto DEFAULT_RTP_JITTER = std::chrono::milliseconds(500);

        inline auto createRtpToRawSrc(std::uint16_t port, Codec codec, std::chrono::milliseconds rtpJitter = DEFAULT_RTP_JITTER) -> std::string {
            std::string parser;
            if (codec == Codec::H265) {
                parser = "! h265parse";
            } else if (codec == Codec::H264) {
                parser = "! h264parse";
            }

            return std::format("udpsrc port={} ! application/x-rtp,media=video ! rtpjitterbuffer latency={} ! {} {} ! decodebin", port, rtpJitter.count(), getRtpDepayloader(codec), parser);
        }

        namespace v4l2 {

            // Based on FOURCC names given by v4l2-ctl --list-formats-ext
#define FORMAT_ITER(_F)       \
    _F(YUYV, RawFormat::YUY2) \
    _F(MJPG, Codec::JPEG)     \
    _F(ABGR32, RawFormat::BGRA)

            enum class PixelFormat : unsigned int {
#define F(name, ...) name,
                FORMAT_ITER(F)
#undef F
            };

            struct CaptureFormat {
                PixelFormat pixelFormat;
                std::uint16_t width;
                std::uint16_t height;
                std::uint16_t framerate;
            };

            constexpr auto toString(PixelFormat pixelFormat) -> std::string_view {
                if (auto name = magic_enum::enum_name(pixelFormat); !name.empty()) {
                    return name;
                }
                throw std::invalid_argument("Unsupported V4L2 format");
            }

            constexpr auto toStringGstType(PixelFormat pixelFormat) -> std::string_view {
                switch (pixelFormat) {
#define F(name, gstType)    \
    case PixelFormat::name: \
        return toString(gstType);

                    FORMAT_ITER(F)
#undef F
                    default:
                        throw std::invalid_argument("Unsupported V4L2 format");
                }
            }

            constexpr auto getPixelFormatFromStringView(std::string_view name) -> PixelFormat {
                if (auto pixelFormat = magic_enum::enum_cast<PixelFormat>(name); pixelFormat.has_value()) {
                    return pixelFormat.value();
                }
                throw std::invalid_argument("Unsupported V4L2 format");
            }

            constexpr auto isRawPixelFormat(PixelFormat pixelFormat) -> bool {
                switch (pixelFormat) {
#define F(name, gstType)    \
    case PixelFormat::name: \
        return std::is_same_v<decltype(gstType), RawFormat>;

                    FORMAT_ITER(F)
#undef F
                    default:
                        throw std::invalid_argument("Unsupported V4L2 format");
                }
            }

            constexpr auto isCompressedPixelFormat(PixelFormat pixelFormat) -> bool {
                switch (pixelFormat) {
#define F(name, gstType)    \
    case PixelFormat::name: \
        return std::is_same_v<decltype(gstType), Codec>;

                    FORMAT_ITER(F)
#undef F
                    default:
                        throw std::invalid_argument("Unsupported V4L2 format");
                }
            }

            constexpr auto getMediaType(PixelFormat pixelFormat) -> std::string_view {
                switch (pixelFormat) {
#define F(name, gstType)    \
    case PixelFormat::name: \
        return getMediaType(gstType);

                    FORMAT_ITER(F)
#undef F
                    default:
                        throw std::invalid_argument("Unsupported V4L2 format");
                }
            }
#undef FORMAT_ITER

            //     Only in GStreamer >1.22 sadge
            //     inline auto addCropProperty(std::uint16_t left, std::uint16_t right, std::uint16_t top, std::uint16_t bottom) -> std::string {
            //         return std::format("crop-left={} crop-right={} crop-top={} crop-bottom={}", left, right, top, bottom);
            //     }

            template<typename... Args>
            inline auto createSrc(std::string_view device, CaptureFormat captureFormat,
                                  Args&&... extraProps) -> std::string
                requires(std::convertible_to<std::remove_cvref_t<Args>, std::string_view> && ...)
            {
                std::ostringstream oss;

                oss << "v4l2src device=" << device;
                ((oss << ' ' << std::forward<Args>(extraProps)), ...);

                oss << " ! " << getMediaType(captureFormat.pixelFormat)
                    << ",width=" << captureFormat.width
                    << ",height=" << captureFormat.height
                    << ",framerate=" << captureFormat.framerate << "/1";

                if (isRawPixelFormat(captureFormat.pixelFormat)) {
                    oss << ",format=" << toStringGstType(captureFormat.pixelFormat);
                }

                return oss.str();
            }
        } // namespace v4l2
    } // namespace video
} // namespace mrover::gst
