#include "message.hpp"

#include <algorithm>
#include <utility>

namespace mrover::dbc_runtime {

    [[nodiscard]] auto CanMessageDescription::name() const -> std::string { return m_name; }
    void CanMessageDescription::set_name(std::string&& name) { m_name = name; }
    void CanMessageDescription::set_name(std::string_view name) { m_name = name; }

    [[nodiscard]] auto CanMessageDescription::id() const -> uint32_t { return m_id; }
    void CanMessageDescription::set_id(uint32_t id) { m_id = id; }

    [[nodiscard]] auto CanMessageDescription::length() const -> uint8_t { return m_length; }
    void CanMessageDescription::set_length(uint8_t length) { m_length = length; }

    [[nodiscard]] auto CanMessageDescription::transmitter() const -> std::string { return m_transmitter; }
    void CanMessageDescription::set_transmitter(std::string&& transmitter) { m_transmitter = transmitter; }
    void CanMessageDescription::set_transmitter(std::string_view transmitter) { m_transmitter = std::string(transmitter); }

    [[nodiscard]] auto CanMessageDescription::signals_size() const -> std::size_t {
        return m_signals.size();
    }

    auto CanMessageDescription::signal(std::string_view name) -> CanSignalDescription* {
        auto it = m_signals.find(name);
        if (it == m_signals.end()) {
            return nullptr;
        }
        return &it->second;
    }
    auto CanMessageDescription::signal(std::string_view name) const -> CanSignalDescription const* {
        auto it = m_signals.find(name);
        if (it == m_signals.end()) {
            return nullptr;
        }
        return &it->second;
    }

    void CanMessageDescription::clear_signals() {
        m_signals.clear();
    }

    auto CanMessageDescription::add_signal(CanSignalDescription signal) -> CanSignalDescription* {
        auto [it, _] = m_signals.insert_or_assign(std::move(signal.name()), std::move(signal));
        return &it->second;
    }

    [[nodiscard]] auto CanMessageDescription::comment() const -> std::string { return m_comment; }
    void CanMessageDescription::set_comment(std::string&& comment) { m_comment = std::move(comment); }
    void CanMessageDescription::set_comment(std::string_view comment) { m_comment = comment; }

    [[nodiscard]] auto CanMessageDescription::is_valid() const -> bool {
        if (m_name.empty() ||
            m_signals.empty() ||
            std::ranges::none_of(signals(), [](auto const& s) {
                return s.is_valid();
            })) {
            return false;
        }

        uint16_t total_signal_bits = 0;
        for (auto const& signal: signals()) {
            if (signal.bit_start() + signal.bit_length() > m_length * 8) {
                return false;
            }
            total_signal_bits += signal.bit_length();
        }
        if (total_signal_bits > m_length * 8) {
            return false;
        }

        return true;
    }

    auto operator<<(std::ostream& os, CanMessageDescription const& message) -> std::ostream& {
        os << "Message Name: " << message.m_name << "\n";
        os << "Message ID: " << message.m_id << "\n";
        os << "Message Length: " << static_cast<uint32_t>(message.m_length) << " bytes\n";
        os << "Transmitter: " << message.m_transmitter << "\n";
        os << "Comment: \"" << message.m_comment << "\"\n";
        os << "Signals:\n";
        for (auto const& signal: message.signals()) {
            os << signal << "\n";
        }
        return os;
    }

} // namespace mrover::dbc_runtime
