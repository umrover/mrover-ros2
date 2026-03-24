#pragma once

#include <cstdint>
#include <expected>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

#include "message.hpp"

namespace mrover::dbc_runtime {

    struct CanFrame {
        uint32_t id;
        std::vector<uint8_t> data;
    };

    class CanFrameProcessor {
#define FOREACH_ERROR(ERROR)         \
    ERROR(None)                      \
    ERROR(InvalidDataFrame)          \
    ERROR(InvalidMessageDescription) \
    ERROR(InvalidMessageName)        \
    ERROR(InvalidSignalDescription)  \
    ERROR(InvalidSignalName)         \
    ERROR(InvalidSignalValue)

#define GENERATE_ENUM(e) e,
#define GENERATE_STRING(e) #e,

    public:
        enum class Error {
            FOREACH_ERROR(GENERATE_ENUM)
        };

        CanFrameProcessor() = default;

        void add_message_description(CanMessageDescription message);

        auto decode(uint32_t id, std::string_view data) -> std::unordered_map<std::string, CanSignalValue>;
        auto encode(std::string const& message_name, std::unordered_map<std::string, CanSignalValue> const& signal_values) -> std::expected<CanFrame, Error>;

        static constexpr auto to_string(Error e) -> std::string_view {
            constexpr std::string_view names[] = {
                    FOREACH_ERROR(GENERATE_STRING)};
            return names[static_cast<int>(e)];
        }

        friend auto operator<<(std::ostream& os, Error e) -> std::ostream& {
            return os << to_string(e);
        }

    private:
        static auto extract_raw_bytes(std::string_view data, CanSignalDescription const& signal) -> std::expected<std::vector<uint8_t>, Error>;
        static auto from_signal_description(std::string_view data, CanSignalDescription const& signal) -> std::expected<CanSignalValue, Error>;

        std::unordered_map<uint32_t, CanMessageDescription> m_message_descriptions{};

#undef GENERATE_ENUM
#undef GENERATE_STRING
#undef FOREACH_ERROR
    };
} // namespace mrover::dbc_runtime
