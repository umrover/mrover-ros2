#pragma once

#include <format>
#include <stdexcept>
#include <string>
#include <string_view>

#include <magic_enum.hpp>
#include <type_traits>

#include "gst.hpp"

namespace mrover::gst::Video::V4L2 {

    // Based on names given by v4l2-ctl --list-formats-ext
#define FORMAT_ITER(_F)       \
    _F(YUYV, RawFormat::YUY2) \
    _F(MJPG, Codec::JPEG)

    enum class Format : unsigned int {
#define F(name, ...) name,
        FORMAT_ITER(F)
#undef F
    };

    constexpr auto toString(Format format) -> std::string_view {
        if (auto name = magic_enum::enum_name(format); !name.empty()) {
            return name;
        }
        throw std::invalid_argument("Unsupported V4L2 format");
    }

    constexpr auto getFormatFromStringView(std::string_view name) -> Format {
        if (auto format = magic_enum::enum_cast<Format>(name); format.has_value()) {
            return format.value();
        }
        throw std::invalid_argument("Unsupported V4L2 format");
    }

    constexpr auto isRawFormat(Format format) -> bool {
        switch (format) {
#define F(name, gstType) \
    case Format::name:   \
        return std::is_same_v<decltype(gstType), RawFormat>;

            FORMAT_ITER(F)
#undef F
            default:
                throw std::invalid_argument("Unsupported V4L2 format");
        }
    }

    constexpr auto isCompressedFormat(Format format) -> bool {
        switch (format) {
#define F(name, gstType) \
    case Format::name:   \
        return std::is_same_v<decltype(gstType), Codec>;

            FORMAT_ITER(F)
#undef F
            default:
                throw std::invalid_argument("Unsupported V4L2 format");
        }
    }

    constexpr auto getMediaType(Format format) -> std::string_view {
        switch (format) {
#define F(name, gstType) \
    case Format::name:   \
        return getMediaType(gstType);

            FORMAT_ITER(F)
#undef F
            default:
                throw std::invalid_argument("Unsupported V4L2 format");
        }
    }

    inline auto addProperty(std::string_view key, auto value) -> std::string {
        return std::format("{}={}", key, value);
    }

    //     Only in GStreamer >1.22 sadge
    //     inline auto addCropProperty(std::uint16_t left, std::uint16_t right, std::uint16_t top, std::uint16_t bottom) -> std::string {
    //         return std::format("crop-left={} crop-right={} crop-top={} crop-bottom={}", left, right, top, bottom);
    //     }

    template<typename... Args>
    auto createSrc(std::string_view device, Format format,
                   std::uint16_t width, std::uint16_t height, std::uint16_t framerate,
                   Args&&... extraProps) -> std::string {

        std::string v4l2src = std::format("v4l2src device={}", device);

        ((v4l2src += std::format(" {}", std::forward<Args>(extraProps))), ...);

        auto base = std::format("{} ! {},width={},height={},framerate={}/1",
                                v4l2src, getMediaType(format), width, height, framerate);

        if (isRawFormat(format)) {
            return std::format("{},format={}", base, toString(format));
        }

        return base;
    }


#undef FORMAT_ITER

} // namespace mrover::gst::Video::V4L2
