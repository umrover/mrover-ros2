#include "file_parser.hpp"

#include <charconv>
#include <concepts>
#include <expected>
#include <filesystem>
#include <fstream>
#include <ios>
#include <string>
#include <string_view>
#include <system_error>
#include <vector>

namespace mrover::dbc_runtime {
    using std::string_view;

    static constexpr auto MESSAGE_HEADER = "BO_ ";
    static constexpr auto SIGNAL_HEADER = "SG_ ";
    static constexpr auto SIGNAL_TYPE_HEADER = "SIG_VALTYPE_ ";
    static constexpr auto COMMENT_HEADER = "CM_ ";
    static constexpr auto SYMBOLS_HEADER = "NS_ ";

    namespace {
        constexpr inline auto trim_back(std::string_view sv) -> std::string_view {
            auto end = sv.find_last_not_of(" \t\n\r\f\v");
            if (end == std::string_view::npos) {
                sv.remove_suffix(sv.size());
            } else {
                sv.remove_suffix(sv.size() - end - 1);
            }
            return sv;
        }

        constexpr inline auto trim_front(std::string_view sv) -> std::string_view {
            auto start = sv.find_first_not_of(" \t\n\r\f\v");
            if (start == std::string_view::npos) {
                sv.remove_prefix(sv.size());
            } else {
                sv.remove_prefix(start);
            }
            return sv;
        }

        constexpr inline auto trim(std::string_view sv) -> std::string_view {
            sv = trim_front(sv);
            sv = trim_back(sv);
            return sv;
        }

        inline void strip_utf8_bom_if_present(std::string_view& sv) noexcept {
            if (sv.size() >= 3 &&
                static_cast<unsigned char>(sv[0]) == 0xEF &&
                static_cast<unsigned char>(sv[1]) == 0xBB &&
                static_cast<unsigned char>(sv[2]) == 0xBF) {
                sv.remove_prefix(3);
            }
        }

        auto next_word(string_view& sv) -> string_view {
            size_t start = 0;
            while (start < sv.size() && std::isspace(static_cast<unsigned char>(sv[start]))) {
                ++start;
            }
            sv.remove_prefix(start);

            if (sv.empty()) {
                return {};
            }

            size_t end = 0;
            while (end < sv.size() && !std::isspace(static_cast<unsigned char>(sv[end]))) {
                ++end;
            }

            string_view word = sv.substr(0, end);

            sv.remove_prefix(end);

            size_t next_start = 0;
            while (next_start < sv.size() && std::isspace(static_cast<unsigned char>(sv[next_start]))) {
                ++next_start;
            }
            sv.remove_prefix(next_start);

            return word;
        }

        auto next_line(string_view& sv) -> string_view {
            if (sv.empty()) return {};

            size_t end = sv.find('\n');
            if (end == string_view::npos) {
                string_view line = sv;
                sv.remove_prefix(sv.size());
                return line;
            }

            string_view line = sv.substr(0, end + 1);
            sv.remove_prefix(end + 1);

            return line;
        }

        template<std::integral T>
            requires(!std::same_as<T, bool>)
        [[nodiscard]] auto to_int(string_view sv, int base = 10) noexcept -> std::expected<T, std::errc> {
            if (base < 2 || base > 36) {
                return std::unexpected(std::errc::invalid_argument);
            }

            T value{};
            char const* first = sv.data();
            char const* last = sv.data() + sv.length();

            std::from_chars_result res = std::from_chars(first, last, value, base);

            if (res.ec != std::errc()) return std::unexpected(res.ec);
            if (res.ptr != last) return std::unexpected(std::errc::invalid_argument);

            return std::expected<T, std::errc>(std::in_place, value);
        }

        template<std::floating_point T>
        [[nodiscard]] auto to_floating_point(string_view sv) noexcept -> std::expected<T, std::errc> {
            T value{};
            char const* first = sv.data();
            char const* last = sv.data() + sv.length();

            std::from_chars_result res = std::from_chars(first, last, value);

            if (res.ec != std::errc()) return std::unexpected(res.ec);
            if (res.ptr != last) return std::unexpected(std::errc::invalid_argument);

            return std::expected<T, std::errc>(std::in_place, value);
        }

    } // namespace

    [[nodiscard]] auto CanDbcFileParser::lines_parsed() const -> std::size_t { return m_lines_parsed; }

    [[nodiscard]] auto CanDbcFileParser::is_error() const -> bool { return m_error != Error::None; }
    [[nodiscard]] auto CanDbcFileParser::error() const -> Error { return m_error; }

    [[nodiscard]] auto CanDbcFileParser::message(uint32_t id) -> CanMessageDescription* {
        if (auto it = m_messages.find(id); it != m_messages.end())
            return std::addressof(it->second);

        if (m_is_processing_message && m_current_message.id() == id)
            return std::addressof(m_current_message);

        return nullptr;
    }

    [[nodiscard]] auto CanDbcFileParser::message(uint32_t id) const -> CanMessageDescription const* {
        if (auto it = m_messages.find(id); it != m_messages.end())
            return std::addressof(it->second);

        if (m_is_processing_message && m_current_message.id() == id)
            return std::addressof(m_current_message);

        return nullptr;
    }

    [[nodiscard]] auto CanDbcFileParser::message(std::string_view name) -> CanMessageDescription* {
        for (auto& [_, message]: m_messages) {
            if (message.name() == name) {
                return std::addressof(message);
            }
        }

        if (m_is_processing_message && m_current_message.name() == name) {
            return std::addressof(m_current_message);
        }

        return nullptr;
    }
    [[nodiscard]] auto CanDbcFileParser::message(std::string_view name) const -> CanMessageDescription const* {
        for (auto& [_, message]: m_messages) {
            if (message.name() == name) {
                return std::addressof(message);
            }
        }

        if (m_is_processing_message && m_current_message.name() == name) {
            return std::addressof(m_current_message);
        }

        return nullptr;
    }

    auto CanDbcFileParser::parse(std::string const& filepath) -> bool {
        std::string file;
        std::error_code ec;
        auto size = std::filesystem::file_size(filepath, ec);
        if (ec) {
            m_error = Error::FileRead;
            return false;
        }

        std::ifstream file_stream(filepath, std::ios::binary);
        if (!file_stream.is_open()) {
            m_error = Error::FileRead;
            return false;
        }

        file.resize(static_cast<std::size_t>(size));
        if (size != 0) {
            file_stream.read(file.data(), static_cast<std::streamsize>(size));

            if (file_stream.bad()) {
                m_error = Error::FileRead;
                return false;
            }
            if (file_stream.fail() && !file_stream.eof()) {
                m_error = Error::FileRead;
                return false;
            }

            // didn't read expected number of bytes
            if (file_stream.gcount() != size) {
                m_error = Error::FileRead;
                return false;
            }
        }

        reset();

        return process_file(file);
    }

    void CanDbcFileParser::reset() {
        m_messages.clear();
        m_current_message = CanMessageDescription{};
        m_is_processing_message = false;
        m_lines_parsed = 0;
        m_error = Error::None;
    }

    auto CanDbcFileParser::process_file(string_view file_view) -> bool {
        strip_utf8_bom_if_present(file_view);

        std::vector<CommentAttribute> comments;
        std::vector<SignalValueTypeAttribute> signal_value_types;

        m_lines_parsed = 0;
        while (!file_view.empty()) {
            string_view line = trim_front(next_line(file_view));
            ++m_lines_parsed;

            if (line.empty() || line.starts_with("//")) {
                continue;
            } else if (line.starts_with(MESSAGE_HEADER)) {
                if (m_is_processing_message) {
                    if (!add_current_message()) {
                        m_error = Error::InvalidMessageFormat;
                        return false;
                    }
                }
                m_is_processing_message = true;

                auto message = parse_message(line);
                if (!message.has_value()) {
                    m_error = message.error();
                    return false;
                }
                m_current_message = std::move(message.value());
            } else if (line.starts_with(SIGNAL_HEADER)) {
                if (!m_is_processing_message) {
                    m_error = Error::InvalidSignalFormat;
                    return false;
                }

                auto signal = parse_signal(line);
                if (!signal.has_value()) {
                    m_error = signal.error();
                    return false;
                }
                m_current_message.add_signal(signal.value());
            } else if (line.starts_with(COMMENT_HEADER)) {
                line.remove_prefix(std::string_view(COMMENT_HEADER).size());

                string_view target_type = next_word(line);
                if (target_type != "BO_" && target_type != "SG_") {
                    m_error = Error::InvalidCommentType;
                    return false;
                }

                // ===== ID =====
                string_view id_str = next_word(line);
                auto id_result = to_int<uint32_t>(id_str);
                if (!id_result.has_value()) {
                    m_error = Error::InvalidCommentMessageId;
                    return false;
                }
                uint32_t id = id_result.value();

                std::optional<std::string> signal_name;
                if (target_type == "SG_") {
                    string_view signal_name_str = next_word(line);
                    if (signal_name_str.empty()) {
                        m_error = Error::InvalidCommentSignalName;
                        return false;
                    }
                    signal_name = std::string(signal_name_str);
                }

                // ===== COMMENT TEXT=====
                string_view cur = line;
                if (cur.empty() || cur.front() != '"') {
                    m_error = Error::InvalidCommentText;
                    return false;
                }
                cur.remove_prefix(1);

                std::string text;
                std::size_t comment_lines_parsed = 0;

                auto ends_with_closer = [](std::string_view sv) -> int {
                    sv = trim(sv);
                    if (sv.size() >= 2 && sv[sv.size() - 2] == '"' && sv.back() == ';') {
                        return 2;
                    }
                    if (!sv.empty() && sv.back() == '"') {
                        return 1;
                    }
                    return 0;
                };

                for (;;) {
                    int strip = ends_with_closer(cur);
                    if (strip > 0) {
                        cur = trim_back(cur);
                        cur.remove_suffix(strip);
                        text.append(cur.begin(), cur.end());
                        break;
                    }

                    text.append(cur.begin(), cur.end());
                    if (file_view.empty()) {
                        m_error = Error::InvalidCommentText;
                        return false;
                    }
                    ++comment_lines_parsed;
                    cur = next_line(file_view);
                }

                comments.push_back(CommentAttribute{
                        .message_id = id,
                        .signal_name = signal_name,
                        .text = text,
                });
                m_lines_parsed += comment_lines_parsed;
            } else if (line.starts_with(SIGNAL_TYPE_HEADER)) {
                auto svt = parse_signal_value_type(line);
                if (!svt.has_value()) {
                    m_error = svt.error();
                    return false;
                }
                signal_value_types.emplace_back(std::move(svt.value()));
            }
        } // while (!file_view.empty())

        if (m_is_processing_message) {
            if (!add_current_message()) {
                return false;
            }
        }

        for (auto const& comment: comments) {
            CanMessageDescription* msg = message(comment.message_id);

            if (msg == nullptr) {
                m_error = Error::InvalidCommentMessageId;
                return false;
            }

            if (comment.signal_name.has_value()) {
                auto* signal_desc = msg->signal(comment.signal_name.value());
                if (signal_desc == nullptr) {
                    m_error = Error::InvalidCommentSignalName;
                    return false;
                }
                signal_desc->set_comment(comment.text);
            } else {
                msg->set_comment(comment.text);
            }
        }

        for (auto const& svt: signal_value_types) {
            CanMessageDescription* msg = message(svt.message_id);

            if (msg == nullptr) {
                m_error = Error::InvalidSignalTypeMessageId;
                return false;
            }

            auto* signal_desc = msg->signal(svt.signal_name);
            if (signal_desc == nullptr) {
                m_error = Error::InvalidSignalTypeSignalName;
                return false;
            }
            signal_desc->set_data_format(svt.data_format);

            if (!signal_desc->is_valid()) {
                m_error = Error::InvalidSignalTypeFormat;
                return false;
            }
        }

        return true;
    }

    auto CanDbcFileParser::parse_message(string_view line) -> std::expected<CanMessageDescription, Error> {
        line = trim(line);
        if (!line.starts_with(MESSAGE_HEADER)) {
            return std::unexpected(Error::InvalidMessageFormat);
        }

        line.remove_prefix(std::string_view(MESSAGE_HEADER).size());

        CanMessageDescription message;
        // ===== ID =====
        string_view id = next_word(line);
        auto id_result = to_int<uint32_t>(id);
        if (!id_result.has_value()) {
            return std::unexpected(Error::InvalidMessageId);
        }
        message.set_id(id_result.value());

        // ===== NAME =====
        string_view name = next_word(line);
        if (!name.ends_with(':')) {
            return std::unexpected(Error::InvalidMessageName);
        }
        name.remove_suffix(1);
        message.set_name(name);

        // ===== LENGTH =====
        string_view length = next_word(line);
        auto length_result = to_int<uint8_t>(length);
        if (!length_result.has_value()) {
            return std::unexpected(Error::InvalidMessageLength);
        }
        message.set_length(length_result.value());

        // ===== TRANSMITTER =====
        string_view transmitter = next_word(line);
        if (transmitter.empty()) {
            return std::unexpected(Error::InvalidMessageTransmitter);
        }
        message.set_transmitter(transmitter);

        if (!line.empty()) {
            return std::unexpected(Error::InvalidMessageFormat);
        }

        return std::expected<CanMessageDescription, Error>(std::in_place, std::move(message));
    }

    auto CanDbcFileParser::parse_signal(string_view line) -> std::expected<CanSignalDescription, Error> {
        line = trim(line);
        if (!line.starts_with(SIGNAL_HEADER)) {
            return std::unexpected(Error::InvalidSignalFormat);
        }

        line.remove_prefix(std::string_view(SIGNAL_HEADER).size());

        // ===== SIGNAL NAME =====
        CanSignalDescription signal;
        string_view name = next_word(line);
        if (name.empty()) {
            return std::unexpected(Error::InvalidSignalName);
        }
        signal.set_name(name);

        // ===== MULTIPLEX =====
        string_view colon_or_multiplex = next_word(line);
        if (colon_or_multiplex == ":") {
            signal.set_multiplex_state(MultiplexState::None);
        } else if (colon_or_multiplex == "M") {
            signal.set_multiplex_state(MultiplexState::MultiplexorSwitch);
            string_view colon = next_word(line);
            if (colon != ":") {
                return std::unexpected(Error::InvalidSignalFormat);
            }
        } else if (colon_or_multiplex.starts_with("m")) {
            signal.set_multiplex_state(MultiplexState::MultiplexedSignal);
            string_view colon = next_word(line);
            if (colon != ":") {
                return std::unexpected(Error::InvalidSignalFormat);
            }
        } else {
            return std::unexpected(Error::InvalidSignalFormat);
        }

        // ===== BIT INFO =====
        string_view bit_info = next_word(line);
        size_t sep_pos = bit_info.find('@');
        if (sep_pos == std::string_view::npos) {
            return std::unexpected(Error::InvalidSignalBitInfo);
        }
        string_view endianess_and_format = bit_info.substr(sep_pos + 1);
        if (endianess_and_format.size() != 2) {
            return std::unexpected(Error::InvalidSignalBitInfo);
        }
        char endianess_char = endianess_and_format[0];
        if (endianess_char == '0') {
            signal.set_endianness(Endianness::BigEndian);
        } else if (endianess_char == '1') {
            signal.set_endianness(Endianness::LittleEndian);
        } else {
            return std::unexpected(Error::InvalidSignalBitInfo);
        }
        char format_char = endianess_and_format[1];
        if (format_char == '+') {
            signal.set_data_format(DataFormat::UnsignedInteger);
        } else if (format_char == '-') {
            signal.set_data_format(DataFormat::SignedInteger);
        }
        string_view bit_start_end = bit_info.substr(0, sep_pos);
        string_view bit_start_str;
        string_view bit_length_str;
        sep_pos = bit_start_end.find('|');
        if (sep_pos == std::string_view::npos) {
            return std::unexpected(Error::InvalidSignalBitInfo);
        }
        bit_start_str = bit_start_end.substr(0, sep_pos);
        bit_length_str = bit_start_end.substr(sep_pos + 1);
        auto bit_start_result = to_int<uint16_t>(bit_start_str);
        if (!bit_start_result.has_value()) {
            return std::unexpected(Error::InvalidSignalBitInfo);
        }
        signal.set_bit_start(bit_start_result.value());
        auto bit_length_result = to_int<uint16_t>(bit_length_str);
        if (!bit_length_result.has_value()) {
            return std::unexpected(Error::InvalidSignalBitInfo);
        }
        signal.set_bit_length(bit_length_result.value());

        // ===== FACTOR & OFFSET =====
        string_view factor_and_offset = next_word(line);
        string_view min_and_max;
        string_view unit;
        string_view receiver;
        if (factor_and_offset.starts_with('(') && factor_and_offset.ends_with(')')) {
            factor_and_offset.remove_prefix(1);
            factor_and_offset.remove_suffix(1);
            sep_pos = factor_and_offset.find(',');
            if (sep_pos == std::string_view::npos) {
                return std::unexpected(Error::InvalidSignalFactorOffset);
            }
            string_view factor_str = factor_and_offset.substr(0, sep_pos);
            string_view offset_str = factor_and_offset.substr(sep_pos + 1);
            auto factor = to_floating_point<double>(factor_str);
            if (!factor.has_value()) {
                return std::unexpected(Error::InvalidSignalFactorOffset);
            }
            auto offset = to_floating_point<double>(offset_str);
            if (!offset.has_value()) {
                return std::unexpected(Error::InvalidSignalFactorOffset);
            }
            if (factor.value() == 1.0 && offset.value() == 0.0) {
                signal.clear_factor_offset();
            } else {
                signal.set_factor(factor.value());
                signal.set_offset(offset.value());
            }

            min_and_max = next_word(line);
        } else {
            signal.clear_factor_offset();
            min_and_max = factor_and_offset;
        }

        // ===== MIN & MAX =====
        if (min_and_max.starts_with('[') && min_and_max.ends_with(']')) {
            min_and_max.remove_prefix(1);
            min_and_max.remove_suffix(1);
            sep_pos = min_and_max.find('|');
            if (sep_pos == std::string_view::npos) {
                return std::unexpected(Error::InvalidSignalMinMax);
            }
            string_view min_str = min_and_max.substr(0, sep_pos);
            string_view max_str = min_and_max.substr(sep_pos + 1);
            auto min = to_floating_point<double>(min_str);
            if (!min.has_value()) {
                return std::unexpected(Error::InvalidSignalMinMax);
            }
            auto max = to_floating_point<double>(max_str);
            if (!max.has_value()) {
                return std::unexpected(Error::InvalidSignalMinMax);
            }

            if (min.value() == 0.0 && max.value() == 0.0) {
                signal.clear_minimum_maximum();
            } else {
                signal.set_minimum(min.value());
                signal.set_maximum(max.value());
            }

            unit = next_word(line);
        } else {
            signal.clear_minimum_maximum();
            unit = min_and_max;
        }

        // ===== UNIT =====
        if (unit.starts_with('"') && unit.ends_with('"')) {
            unit.remove_prefix(1);
            unit.remove_suffix(1);
            signal.set_unit(unit);

            receiver = next_word(line);
        } else {
            receiver = unit;
        }

        // ===== RECEIVER =====
        if (!receiver.empty()) {
            signal.set_receiver(receiver);
        }

        if (!line.empty() || !signal.is_valid()) {
            return std::unexpected(Error::InvalidSignalFormat);
        }

        return std::expected<CanSignalDescription, Error>(std::in_place, signal);
    }

    auto CanDbcFileParser::parse_signal_value_type(string_view line) -> std::expected<SignalValueTypeAttribute, Error> {
        line = trim(line);
        if (!line.starts_with(SIGNAL_TYPE_HEADER)) {
            return std::unexpected(Error::InvalidSignalTypeFormat);
        }
        line.remove_prefix(std::string_view(SIGNAL_TYPE_HEADER).size());

        SignalValueTypeAttribute svt;

        // ===== ID =====
        string_view id_str = next_word(line);
        auto id_result = to_int<uint32_t>(id_str);
        if (!id_result.has_value()) {
            return std::unexpected(Error::InvalidSignalTypeMessageId);
        }
        svt.message_id = id_result.value();

        // ===== SIGNAL NAME =====
        string_view signal_name_str = next_word(line);
        if (signal_name_str.empty()) {
            return std::unexpected(Error::InvalidSignalTypeSignalName);
        }
        svt.signal_name = std::string(signal_name_str);

        // ===== TYPE =====
        string_view colon = next_word(line);
        if (colon != ":") {
            return std::unexpected(Error::InvalidSignalTypeFormat);
        }

        string_view type_str = next_word(line);
        if (type_str.empty()) {
            return std::unexpected(Error::InvalidSignalTypeDataType);
        }
        size_t semicolon_pos = type_str.find(';');
        if (semicolon_pos == std::string_view::npos) {
            return std::unexpected(Error::InvalidSignalTypeFormat);
        }
        type_str.remove_suffix(type_str.size() - semicolon_pos);

        if (type_str == "1") {
            svt.data_format = DataFormat::Float;
        } else if (type_str == "2") {
            svt.data_format = DataFormat::Double;
        } else {
            return std::unexpected(Error::InvalidSignalTypeFormat);
        }

        if (!line.empty()) {
            return std::unexpected(Error::InvalidSignalTypeFormat);
        }

        return std::expected<SignalValueTypeAttribute, Error>(std::in_place, svt);
    }

    auto CanDbcFileParser::add_current_message() -> bool {
        if (m_is_processing_message) {
            if (!m_current_message.is_valid()) {
                return false;
            }
            uint32_t const id = m_current_message.id();
            m_messages.emplace(id, std::move(m_current_message));
            m_current_message = {};
            return true;
        }
        return false;
    }


} // namespace mrover::dbc_runtime
