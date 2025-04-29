#pragma once

#include <chrono>
#include <cstdint>
#include <format>
#include <string>

#include "gst_codec.hpp"

namespace mrover {
    namespace gst {
        namespace Video {

            inline auto createRtpSink(std::string const& host, std::uint16_t port, Video::Codec codec) {
                return std::format("{} ! udpsink host={} port={}", getRtpPayloader(codec), host, port);
            }

            constexpr auto DEFAULT_RTP_JITTER = std::chrono::milliseconds(500);

            inline auto createRtpToRawSrc(std::uint16_t port, Video::Codec codec, std::chrono::milliseconds rtpJitter = DEFAULT_RTP_JITTER) -> std::string {
                return std::format("udpsrc port={} ! application/x-rtp,media=video ! rtpjitterbuffer latency={} ! {} ! decodebin", port, rtpJitter.count(), getRtpDepayloader(codec));
            }
        } // namespace Video

    } // namespace gst

} // namespace mrover
