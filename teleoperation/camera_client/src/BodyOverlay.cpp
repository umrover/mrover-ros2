#include "BodyOverlay.hpp"

namespace mrover {

    namespace {
        // Keep enough history to cover the delay plus some slack
        constexpr auto HISTORY_SLACK = std::chrono::seconds{2};
        // A message this much older than the delay no longer describes the picture: treat as no data
        constexpr auto STALE_AFTER = std::chrono::seconds{1};

        // BODY_18 skeleton (sl::BODY_18_PARTS)
        constexpr std::array<std::pair<std::size_t, std::size_t>, 13> BONES{{
                {0, 1},                      // nose - neck
                {1, 2}, {2, 3}, {3, 4},      // right arm
                {1, 5}, {5, 6}, {6, 7},      // left arm
                {1, 8}, {8, 9}, {9, 10},     // right leg
                {1, 11}, {11, 12}, {12, 13}, // left leg
        }};

        QColor const BONE_COLOR{0, 200, 255};
        QColor const CONFIDENT_COLOR{0, 230, 0};
        QColor const UNCONFIDENT_COLOR{230, 0, 0};

    } // namespace

    BodyOverlayBuffer::BodyOverlayBuffer(std::chrono::milliseconds delay) : mDelay{delay} {}

    auto BodyOverlayBuffer::push(msg::BodyOverlay::ConstSharedPtr msg) -> void {
        auto const now = Clock::now();
        std::scoped_lock lock{mMutex};
        mHistory.emplace_back(now, std::move(msg));
        while (mHistory.size() > 1 && mHistory.front().first < now - mDelay - HISTORY_SLACK) {
            mHistory.pop_front();
        }
    }

    auto BodyOverlayBuffer::delayed() -> msg::BodyOverlay::ConstSharedPtr {
        auto const cutoff = Clock::now() - mDelay;
        std::scoped_lock lock{mMutex};
        for (auto it = mHistory.rbegin(); it != mHistory.rend(); ++it) {
            if (it->first <= cutoff) {
                return it->first < cutoff - STALE_AFTER ? nullptr : it->second;
            }
        }
        return nullptr;
    }

    auto drawBodySkeleton(QPainter& painter, QRectF const& frameRect, msg::BodyOverlay const& overlay) -> void {
        painter.save();
        painter.setRenderHint(QPainter::Antialiasing);

        auto const& us = overlay.joints_u;
        auto const& vs = overlay.joints_v;
        auto const& confident = overlay.joints_confident;
        std::size_t const n = std::min(us.size(), vs.size());
        auto visible = [&](std::size_t i) { return i < n && us[i] >= 0 && vs[i] >= 0; };
        auto point = [&](std::size_t i) {
            return QPointF{frameRect.left() + us[i] * frameRect.width(), frameRect.top() + vs[i] * frameRect.height()};
        };

        qreal const scale = std::max(frameRect.width(), frameRect.height()) / 640.0;
        qreal const boneWidth = std::max(2.0, 3.0 * scale);
        qreal const jointRadius = std::max(3.0, 4.0 * scale);

        painter.setPen(QPen{BONE_COLOR, boneWidth, Qt::SolidLine, Qt::RoundCap});
        for (auto const& [a, b]: BONES) {
            if (visible(a) && visible(b)) painter.drawLine(point(a), point(b));
        }

        painter.setPen(Qt::NoPen);
        for (std::size_t i = 0; i < n; ++i) {
            if (!visible(i)) continue;
            bool const isConfident = i < confident.size() && confident[i];
            painter.setBrush(isConfident ? CONFIDENT_COLOR : UNCONFIDENT_COLOR);
            painter.drawEllipse(point(i), jointRadius, jointRadius);
        }

        painter.restore();
    }

    auto drawBodyStatus(QPainter& painter, QRectF const& area, msg::BodyOverlay const* overlay) -> void {
        painter.save();
        painter.setRenderHint(QPainter::Antialiasing);

        QString text;
        QColor color;
        std::optional<float> progress;

        if (!overlay) {
            text = "Body overlay: no data";
            color = Qt::gray;
        } else {
            switch (overlay->state) {
                case msg::BodyOverlay::TRACKING:
                    text = "Person tracked";
                    color = Qt::white;
                    break;
                case msg::BodyOverlay::HOLDING:
                    text = QString{"T-pose %1 / %2 s"}.arg(overlay->hold_elapsed, 0, 'f', 1).arg(overlay->hold_required, 0, 'f', 1);
                    color = Qt::yellow;
                    if (overlay->hold_required > 0) {
                        progress = std::clamp(overlay->hold_elapsed / overlay->hold_required, 0.0f, 1.0f);
                    }
                    break;
                case msg::BodyOverlay::LOCKED:
                    text = "ASTRONAUT LOCKED";
                    color = CONFIDENT_COLOR;
                    break;
                case msg::BodyOverlay::NO_BODY:
                default:
                    text = "No person";
                    color = Qt::gray;
                    break;
            }
        }

        QFont font = painter.font();
        font.setBold(true);
        // Scale with the shorter side so rotated (portrait) feeds don't get oversized text
        font.setPixelSize(std::max(12, static_cast<int>(std::min(area.width(), area.height()) / 22)));
        painter.setFont(font);

        QFontMetricsF const metrics{font};
        qreal const margin = metrics.height() * 0.3;
        qreal const barHeight = progress ? metrics.height() * 0.25 : 0;
        QRectF box{area.left() + margin, area.top() + margin,
                   metrics.horizontalAdvance(text) + 2 * margin,
                   metrics.height() + 2 * margin + (progress ? barHeight + margin : 0)};

        painter.setPen(Qt::NoPen);
        painter.setBrush(QColor{0, 0, 0, 170});
        painter.drawRoundedRect(box, margin, margin);

        painter.setPen(color);
        painter.drawText(QRectF{box.left() + margin, box.top() + margin, metrics.horizontalAdvance(text), metrics.height()},
                         Qt::AlignLeft | Qt::AlignVCenter, text);

        if (progress) {
            QRectF const bar{box.left() + margin, box.top() + 2 * margin + metrics.height(),
                             box.width() - 2 * margin, barHeight};
            painter.setPen(Qt::NoPen);
            painter.setBrush(QColor{255, 255, 255, 60});
            painter.drawRect(bar);
            painter.setBrush(color);
            painter.drawRect(QRectF{bar.topLeft(), QSizeF{bar.width() * *progress, bar.height()}});
        }

        painter.restore();
    }

} // namespace mrover
