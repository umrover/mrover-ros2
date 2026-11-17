#pragma once

#include "pch.hpp"

#include <QPainter>

namespace mrover {

    /**
     * @brief Recent BodyOverlay messages from the gesture node, so the overlay can be drawn delayed.
     *
     * The joints arrive over ROS within milliseconds, while the video they belong to goes through encoding,
     * the RTP jitter buffer and decoding first. Drawing the message received `delay` ago lines the skeleton
     * up with the picture again. The delay is a parameter tuned by eye (body_overlay.delay_ms).
     */
    class BodyOverlayBuffer {
        using Clock = std::chrono::steady_clock;

        std::mutex mMutex;
        std::deque<std::pair<Clock::time_point, msg::BodyOverlay::ConstSharedPtr>> mHistory;
        std::chrono::milliseconds mDelay;

    public:
        explicit BodyOverlayBuffer(std::chrono::milliseconds delay);

        auto push(msg::BodyOverlay::ConstSharedPtr msg) -> void;

        /**
         * @return The newest message received at least `delay` ago, or nullptr if there is none
         *         or it is too old to still describe the picture (the gesture node stopped, link lost)
         */
        auto delayed() -> msg::BodyOverlay::ConstSharedPtr;
    };

    /**
     * @brief Draw the skeleton (bones + joints) over a video frame
     *
     * Draws in the frame's own coordinates: call it with the same painter transform the frame was drawn with
     * (GstVideoWidget rotates the painter), so the skeleton turns together with the picture.
     *
     * @param frameRect Rect the frame image was drawn into
     */
    auto drawBodySkeleton(QPainter& painter, QRectF const& frameRect, msg::BodyOverlay const& overlay) -> void;

    /**
     * @brief Draw the gesture state badge (tracking / T-pose progress / locked) upright
     *
     * @param area      On-screen rect of the shown frame, in untransformed widget coordinates
     * @param overlay   Message to show, nullptr shows a "no data" notice
     */
    auto drawBodyStatus(QPainter& painter, QRectF const& area, msg::BodyOverlay const* overlay) -> void;

} // namespace mrover
