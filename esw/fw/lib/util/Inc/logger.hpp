#pragma once

#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <string_view>

#include <serial/uart.hpp>

namespace mrover {

    static constexpr size_t LOG_BUFFER_SIZE = 128;

#ifdef HAL_UART_MODULE_ENABLED
    // #ifdef DEBUG
    class Logger {
    public:
        enum class Level : uint8_t {
            Debug = 0,
            Info,
            Warning,
            Error,
            None
        };

        Logger() = delete;

        static auto init(UART* uart, Level const level = Level::Info) -> void {
            s_uart = uart;
            s_level = level;
        }

        static auto instance() -> Logger& {
            static Logger inst{s_level};
            return inst;
        }

        auto set_level(Level const level) -> void {
            m_level = level;
        }

        auto debug(char const* fmt, ...) const -> void {
            va_list args;
            va_start(args, fmt);
            vlog(Level::Debug, "DEBUG: ", fmt, args);
            va_end(args);
        }

        auto info(char const* fmt, ...) const -> void {
            va_list args;
            va_start(args, fmt);
            vlog(Level::Info, "INFO: ", fmt, args);
            va_end(args);
        }

        auto warn(char const* fmt, ...) const -> void {
            va_list args;
            va_start(args, fmt);
            vlog(Level::Warning, "WARN: ", fmt, args);
            va_end(args);
        }

        auto error(char const* fmt, ...) const -> void {
            va_list args;
            va_start(args, fmt);
            vlog(Level::Error, "ERROR: ", fmt, args);
            va_end(args);
        }

    private:
        explicit Logger(Level level) : m_level{level} {}

        auto vlog(
                Level const level,
                std::string_view const prefix,
                char const* fmt,
                va_list const args) const -> void {
            if (!s_uart) return;
            if (level < m_level || m_level == Level::None) return;

            char buffer[LOG_BUFFER_SIZE];
            char tx_buffer[LOG_BUFFER_SIZE + 32];

            int const len = std::vsnprintf(buffer, sizeof(buffer), fmt, args);
            if (len <= 0) return;

            size_t const content_len = std::min(static_cast<size_t>(len), sizeof(buffer) - 1);

            int const tx_len = std::snprintf(
                    tx_buffer,
                    sizeof(tx_buffer),
                    "%.*s%.*s\r\n",
                    static_cast<int>(prefix.size()), prefix.data(),
                    static_cast<int>(content_len), buffer);

            if (tx_len <= 0) return;

            size_t const final_len = std::min(static_cast<size_t>(tx_len), sizeof(tx_buffer) - 1);
            s_uart->transmit(std::string_view{tx_buffer, final_len});
        }

        Level m_level{Level::Info};

        static inline UART* s_uart = nullptr;
        static inline auto s_level = Level::Info;
    };
// #else  // DEBUG
//     class Logger {
//         Logger() = default;
//
//     public:
//         template<typename... Args>
//         static auto init(Args&&... args) -> void {}
//
//         static auto instance() -> Logger& {
//             static Logger inst{};
//             return inst;
//         }
//         template<typename... Args>
//         auto set_level(Args&&... args) -> void {}
//         template<typename... Args>
//         auto debug(Args&&... args) const -> void {}
//         template<typename... Args>
//         auto info(Args&&... args) const -> void {}
//         template<typename... Args>
//         auto warn(Args&&... args) const -> void {}
//         template<typename... Args>
//         auto error(Args&&... args) const -> void {}
//     };
// #endif // DEBUG
#else  // HAL_UART_MODULE_ENABLED
    class Logger {
        Logger() = default;

    public:
        template<typename... Args>
        static auto init(Args&&... args) -> void {}

        static auto instance() -> Logger& {
            static Logger inst{};
            return inst;
        }
        template<typename... Args>
        auto set_level(Args&&... args) -> void {}
        template<typename... Args>
        auto debug(Args&&... args) const -> void {}
        template<typename... Args>
        auto info(Args&&... args) const -> void {}
        template<typename... Args>
        auto warn(Args&&... args) const -> void {}
        template<typename... Args>
        auto error(Args&&... args) const -> void {}
    };
#endif // HAL_UART_MODULE_ENABLED

} // namespace mrover
