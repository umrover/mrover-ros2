#pragma once

#include <chrono>
#include <format>
#include <numeric>
#include <optional>
#include <ranges>
#include <unordered_map>

#include <rclcpp/logger.hpp>

namespace mrover {

    /**
     * @brief Profiles the execution time of a loop composed of multiple events.
     */
    class LoopProfiler {
        using Clock = std::chrono::high_resolution_clock;
        using EventReadings = std::vector<Clock::duration>;
        using DisplayUnits = std::chrono::milliseconds;

        rclcpp::Logger mLogger;
        std::size_t mPrintTick;

        std::unordered_map<std::string, EventReadings> mEventReadings;

        std::optional<Clock::time_point> mLastEpochTime;
        std::size_t mTick = 0; // Loop iteration counter

    public:
        explicit LoopProfiler(rclcpp::Logger&& logger, std::size_t printTick = 120) : mLogger{std::move(logger)}, mPrintTick{printTick} {}

        /**
         * @brief Call this at the beginning of each loop iteration.
         */
        void beginLoop() {
            if (mTick % mPrintTick == 0) {
                Clock::duration averageLoopDuration{};
                for (auto const& durations: mEventReadings | std::views::values | std::views::filter([](auto const& d) { return !d.empty(); })) {
                    averageLoopDuration += std::accumulate(durations.begin(), durations.end(), Clock::duration{}) / durations.size();
                }
                // Print update time for the entire loop
                auto averageLoopMs = std::chrono::duration_cast<DisplayUnits>(averageLoopDuration);
                long hz = averageLoopMs.count() ? DisplayUnits::period::den / averageLoopMs.count() : -1;
                RCLCPP_INFO_STREAM(mLogger, std::format("Total: {}ms ({} Hz)", averageLoopMs.count(), hz));
                // Print update times for each loop event
                for (auto& [name, durations]: mEventReadings | std::views::filter([](auto const& p) { return !p.second.empty(); })) {
                    Clock::duration averageEventDuration = std::accumulate(durations.begin(), durations.end(), Clock::duration{}) / durations.size();
                    auto averageEventMs = std::chrono::duration_cast<DisplayUnits>(averageEventDuration);
                    RCLCPP_INFO_STREAM(mLogger, std::format("\t{}: {}ms", name, averageEventMs.count()));
                    durations.clear();
                }
            }

            mTick++;
        }

        /**
         * @brief Call this at the beginning of each event in the loop.
         *
         * This signals the end of the previous event and starts a new one.
         *
         * @param name
         */
        void measureEvent(std::string const& name) {
            Clock::time_point now = Clock::now();
            if (mLastEpochTime) {
                Clock::duration duration = now - mLastEpochTime.value();
                mEventReadings[name].push_back(duration);
            }
            mLastEpochTime = now;
        }
    };

} // namespace mrover