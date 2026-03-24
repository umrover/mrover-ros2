#pragma once

#include <expected>
#include <ranges>
#include <string>
#include <string_view>
#include <unordered_map>

#include "message.hpp"

/*
    The current implementation supports only a subset of keywords that you can find in a DBC file:
        {BO_} - message description.
        {SG_} - signal description.
        {SIG_VALTYPE_} - signal type description.
        {SG_MUL_VAL_} - extended multiplexing description.
        {CM_} - comments (only for message and signal descriptions) - single line only.
        {VAL_} - textual descriptions for raw signal values.
*/

namespace mrover::dbc_runtime {
    class CanDbcFileParser {

#define FOREACH_ERROR(ERROR)           \
    ERROR(None)                        \
    ERROR(FileRead)                    \
    ERROR(InvalidMessageFormat)        \
    ERROR(InvalidMessageName)          \
    ERROR(InvalidMessageId)            \
    ERROR(InvalidMessageLength)        \
    ERROR(InvalidMessageTransmitter)   \
    ERROR(InvalidSignalFormat)         \
    ERROR(InvalidSignalName)           \
    ERROR(InvalidSignalBitInfo)        \
    ERROR(InvalidSignalFactorOffset)   \
    ERROR(InvalidSignalMinMax)         \
    ERROR(InvalidSignalUnit)           \
    ERROR(InvalidSignalTypeFormat)     \
    ERROR(InvalidSignalTypeMessageId)  \
    ERROR(InvalidSignalTypeSignalName) \
    ERROR(InvalidSignalTypeDataType)   \
    ERROR(InvalidCommentFormat)        \
    ERROR(InvalidCommentType)          \
    ERROR(InvalidCommentMessageId)     \
    ERROR(InvalidCommentSignalName)    \
    ERROR(InvalidCommentText)

#define GENERATE_ENUM(e) e,
#define GENERATE_STRING(e) #e,

    public:
        enum class Error {
            FOREACH_ERROR(GENERATE_ENUM)
        };

        CanDbcFileParser() = default;

        [[nodiscard]] auto lines_parsed() const -> std::size_t;

        [[nodiscard]] auto is_error() const -> bool;
        [[nodiscard]] auto error() const -> Error;

        [[nodiscard]] auto messages() const noexcept { return m_messages | std::views::values; }

        [[nodiscard]] auto message(uint32_t id) -> CanMessageDescription*;
        [[nodiscard]] auto message(uint32_t id) const -> CanMessageDescription const*;
        [[nodiscard]] auto message(std::string_view name) -> CanMessageDescription*;
        [[nodiscard]] auto message(std::string_view name) const -> CanMessageDescription const*;

        auto parse(std::string const& filepath) -> bool;

        static constexpr auto to_string(Error e) -> std::string_view {
            constexpr std::string_view names[] = {
                    FOREACH_ERROR(GENERATE_STRING)};
            return names[static_cast<int>(e)];
        }

        friend auto operator<<(std::ostream& os, Error e) -> std::ostream& {
            return os << to_string(e);
        }

    private:
        std::unordered_map<uint32_t, CanMessageDescription> m_messages{};
        CanMessageDescription m_current_message;
        bool m_is_processing_message = false;
        std::size_t m_lines_parsed = 0;
        Error m_error = Error::None;

        struct CommentAttribute {
            uint32_t message_id;
            std::optional<std::string> signal_name; // nullopt for message comments
            std::string text;
        };

        struct SignalValueTypeAttribute {
            uint32_t message_id;
            std::string signal_name;
            DataFormat data_format;
        };

        void reset();
        auto process_file(std::string_view file_view) -> bool;
        static auto parse_message(std::string_view line) -> std::expected<CanMessageDescription, Error>;
        static auto parse_signal(std::string_view line) -> std::expected<CanSignalDescription, Error>;
        static auto parse_signal_value_type(std::string_view line) -> std::expected<SignalValueTypeAttribute, Error>;

        auto add_current_message() -> bool;

#undef GENERATE_ENUM
#undef GENERATE_STRING
#undef FOREACH_ERROR
    };

} // namespace mrover::dbc_runtime
