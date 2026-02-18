#pragma once

#include <cstdint>
#include <ostream>
#include <ranges>
#include <string>
#include <unordered_map>

#include "signal.hpp"

namespace mrover::dbc_runtime {

    class CanMessageDescription {
    public:
        [[nodiscard]] auto name() const -> std::string;
        void set_name(std::string&& name);
        void set_name(std::string_view name);

        [[nodiscard]] auto id() const -> uint32_t;
        void set_id(uint32_t id);

        [[nodiscard]] auto length() const -> uint8_t;
        void set_length(uint8_t length);

        [[nodiscard]] auto transmitter() const -> std::string;
        void set_transmitter(std::string&& transmitter);
        void set_transmitter(std::string_view transmitter);

        [[nodiscard]] auto signals() noexcept { return m_signals | std::views::values; }
        [[nodiscard]] auto signals() const noexcept { return m_signals | std::views::values; }

        [[nodiscard]] auto signals_size() const -> std::size_t;

        [[nodiscard]] auto signal(std::string_view name) -> CanSignalDescription*;
        [[nodiscard]] auto signal(std::string_view name) const -> CanSignalDescription const*;

        void clear_signals();

        auto add_signal(CanSignalDescription signal) -> CanSignalDescription*;

        [[nodiscard]] auto comment() const -> std::string;
        void set_comment(std::string&& comment);
        void set_comment(std::string_view comment);

        [[nodiscard]] auto is_valid() const -> bool;

        friend auto operator<<(std::ostream& os, CanMessageDescription const& message) -> std::ostream&;

    private:
        struct TransparentHash {
            using is_transparent = void;

            auto operator()(std::string_view sv) const noexcept -> std::size_t { return std::hash<std::string_view>{}(sv); }
            auto operator()(std::string const& s) const noexcept -> std::size_t { return (*this)(std::string_view{s}); }
            auto operator()(char const* s) const noexcept -> std::size_t { return (*this)(std::string_view{s}); }
        };

        struct TransparentEqual {
            using is_transparent = void;

            auto operator()(std::string_view a, std::string_view b) const noexcept -> bool { return a == b; }
        };

        std::string m_name;
        uint32_t m_id;
        uint8_t m_length; // in bytes
        std::string m_transmitter;
        std::unordered_map<std::string, CanSignalDescription, TransparentHash, TransparentEqual> m_signals;
        std::string m_comment;
    };


} // namespace mrover::dbc_runtime
