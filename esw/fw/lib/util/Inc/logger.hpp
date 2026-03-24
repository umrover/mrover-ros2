#pragma once

#include <cstdarg>
#include <cstdint>
#include <string_view>

#include <serial/uart.hpp>

#ifdef STM32
#include "main.h"
#endif // STM32

namespace mrover {

    static constexpr size_t LOG_BUFFER_SIZE = 128;

#ifdef HAL_UART_MODULE_ENABLED
    class Logger {
    public:
        enum class Level : uint8_t {
            Debug = 0,
            Info,
            Warning,
            Error,
            None
        };

        static auto init(UART* uart, Level const level = Level::Info) -> void {
            s_uart = uart;
            s_level = level;
        }

        static auto instance() -> Logger& {
            static Logger inst;
            return inst;
        }

        auto debug(char const* fmt, ...) -> void {
            va_list args;
            va_start(args, fmt);
            vlog(Level::Debug, "DEBUG: ", fmt, args);
            va_end(args);
        }

        auto info(char const* fmt, ...) -> void {
            va_list args;
            va_start(args, fmt);
            vlog(Level::Info, "INFO:  ", fmt, args);
            va_end(args);
        }

        auto warn(char const* fmt, ...) -> void {
            va_list args;
            va_start(args, fmt);
            vlog(Level::Warning, "WARN:  ", fmt, args);
            va_end(args);
        }

        auto error(char const* fmt, ...) -> void {
            va_list args;
            va_start(args, fmt);
            vlog(Level::Error, "ERROR: ", fmt, args);
            va_end(args);
        }

    private:
        char m_buf[LOG_BUFFER_SIZE]{};
        size_t m_idx{0};
        static inline UART* s_uart{nullptr};
        static inline auto s_level{Level::Info};

        Logger() = default;

        auto vlog(Level const level, char const* prefix, char const* fmt, va_list const args) -> void {
            if (!s_uart || level < s_level || s_level == Level::None) return;

            __disable_irq();

            m_idx = 0;
            write_str(prefix);
            format(fmt, args);
            write_str("\r\n");

            s_uart->transmit(std::string_view{m_buf, m_idx});

            __enable_irq();
        }

        /**
         * Write a string to the output buffer.
         *
         * @param str String to buffer
         */
        auto write_str(char const* str) -> void {
            while (*str && m_idx < LOG_BUFFER_SIZE) m_buf[m_idx++] = *str++;
        }

        /**
         * Write unsigned 32-bit integer to buffer.
         *
         * @param val Unsigned integer to write
         */
        auto write_uint(uint32_t val) -> void {
            if (val == 0) {
                if (m_idx < LOG_BUFFER_SIZE) m_buf[m_idx++] = '0';
                return;
            }

            char tmp[10]; // uint32_t max value is 10 digits
            int i = 0;
            while (val > 0) {
                tmp[i++] = (val % 10) + '0';
                val /= 10;
            }

            // write digits in reverse order
            while (i > 0 && m_idx < LOG_BUFFER_SIZE) {
                m_buf[m_idx++] = tmp[--i];
            }
        }

        /**
         * Write integer to buffer.
         *
         * @param val Integer value to write
         */
        auto write_int(int32_t const val) -> void {
            if (val < 0) {
                if (m_idx < LOG_BUFFER_SIZE) m_buf[m_idx++] = '-';
                // avoid overflow on int32 minimum value
                write_uint(static_cast<uint32_t>(-(static_cast<int64_t>(val))));
            } else {
                write_uint(static_cast<uint32_t>(val));
            }
        }

        /**
         * Write an unsigned 32-bit integer to buffer in hexadecimal.
         *
         * @param val Unsigned integer to write
         */
        auto write_hex(uint32_t val) -> void {
            if (val == 0) {
                if (m_idx < LOG_BUFFER_SIZE) m_buf[m_idx++] = '0';
                return;
            }

            char tmp[8]; // 32 bit hex buffer
            int i = 0;

            while (val > 0 && i < 8) {
                auto const hex_digits = "0123456789abcdef";
                tmp[i++] = hex_digits[val & 0xf];
                val >>= 4;
            }

            while (i > 0 && m_idx < LOG_BUFFER_SIZE) {
                m_buf[m_idx++] = tmp[--i];
            }
        }

        /**
         * Write a float to the buffer.
         *
         * Will cap at 5 decimal places.
         *
         * @param val Floating-point value to write
         */
        auto write_float(float const val) -> void {
            // write initial integer (pre-decimal)
            write_int(static_cast<int32_t>(val));

            if (m_idx < LOG_BUFFER_SIZE) m_buf[m_idx++] = '.';

            // determine fractional component
            float const abs_val = (val < 0.0f) ? -val : val;
            float const frac = abs_val - static_cast<float>(static_cast<int32_t>(abs_val));

            // scale to 5 decimal places, +0.5f/100000.0f to handle wrapping
            auto scaled_frac = static_cast<int32_t>(frac * 100000.0f + 0.5f);

            // cap the carry on rounding in the edge case
            if (scaled_frac >= 100000) scaled_frac = 99999;

            // pad with leading zeros
            if (scaled_frac < 10000)
                if (m_idx < LOG_BUFFER_SIZE) m_buf[m_idx++] = '0';
            if (scaled_frac < 1000)
                if (m_idx < LOG_BUFFER_SIZE) m_buf[m_idx++] = '0';
            if (scaled_frac < 100)
                if (m_idx < LOG_BUFFER_SIZE) m_buf[m_idx++] = '0';
            if (scaled_frac < 10)
                if (m_idx < LOG_BUFFER_SIZE) m_buf[m_idx++] = '0';

            write_int(scaled_frac);
        }

        /**
         * Format a string that parses %s, %u, %d, %x, and %f.
         *
         * This will emplace the provided string into the buffer to be logged.
         *
         * @param fmt Format string to buffer out
         * @param args Arguments to emplace to format string
         */
        auto format(char const* fmt, va_list args) -> void {
            while (*fmt && m_idx < LOG_BUFFER_SIZE) {
                if (*fmt == '%') {
                    ++fmt;
                    switch (*fmt) {
                        case 's':
                            write_str(va_arg(args, char*));
                            break;
                        case 'd':
                            write_int(va_arg(args, int32_t));
                            break;
                        case 'u':
                            write_uint(va_arg(args, uint32_t));
                            break;
                        case 'x':
                            write_hex(va_arg(args, uint32_t));
                            break;
                        case 'f':
                            write_float(static_cast<float>(va_arg(args, double)));
                            break;
                        default:
                            // if unknown, just print character verbatim
                            if (m_idx < LOG_BUFFER_SIZE) m_buf[m_idx++] = *fmt;
                            break;
                    }
                } else {
                    m_buf[m_idx++] = *fmt;
                }
                ++fmt;
            }
        }
    };
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
