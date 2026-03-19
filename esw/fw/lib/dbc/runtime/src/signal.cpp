#include "signal.hpp"

namespace mrover::dbc_runtime {

    auto CanSignalValue::is_integral() const -> bool {
        return std::holds_alternative<int8_t>(*this) ||
               std::holds_alternative<uint8_t>(*this) ||
               std::holds_alternative<int16_t>(*this) ||
               std::holds_alternative<uint16_t>(*this) ||
               std::holds_alternative<int32_t>(*this) ||
               std::holds_alternative<uint32_t>(*this) ||
               std::holds_alternative<int64_t>(*this) ||
               std::holds_alternative<uint64_t>(*this);
    }

    auto CanSignalValue::is_floating_point() const -> bool {
        return std::holds_alternative<float>(*this) ||
               std::holds_alternative<double>(*this);
    }

    auto CanSignalValue::is_numeric() const -> bool {
        return is_integral() || is_floating_point();
    }

    auto CanSignalValue::is_string() const -> bool {
        return std::holds_alternative<std::string>(*this);
    }

    auto CanSignalValue::as_signed_integer() const -> int64_t {
        return std::visit([](auto const& v) -> int64_t {
            using T = std::decay_t<decltype(v)>;

            if constexpr (std::is_same_v<T, std::string>) {
                throw std::bad_variant_access{};
            } else {
                static_assert(std::is_arithmetic_v<T>,
                              "Non-numeric type in CanSignalValue");
                return static_cast<int64_t>(v);
            }
        },
                          *this);
    }

    auto CanSignalValue::as_unsigned_integer() const -> uint64_t {
        return std::visit([](auto const& v) -> int64_t {
            using T = std::decay_t<decltype(v)>;

            if constexpr (std::is_same_v<T, std::string>) {
                throw std::bad_variant_access{};
            } else {
                static_assert(std::is_arithmetic_v<T>,
                              "Non-numeric type in CanSignalValue");
                return static_cast<uint64_t>(v);
            }
        },
                          *this);
    }

    auto CanSignalValue::as_double() const -> double {
        return std::visit([](auto const& v) -> double {
            using T = std::decay_t<decltype(v)>;
            if constexpr (std::is_same_v<T, std::string>) {
                throw std::bad_variant_access{};
            } else {
                return static_cast<double>(v);
            }
        },
                          *this);
    }

    auto CanSignalValue::as_string() const -> std::string {
        return std::get<std::string>(*this);
    }

    auto operator<<(std::ostream& os, CanSignalValue const& value) -> std::ostream& {
        std::visit([&os](auto const& v) {
            os << v;
        },
                   value);

        return os;
    }

    [[nodiscard]] auto CanSignalDescription::name() const -> std::string { return m_name; }
    void CanSignalDescription::set_name(std::string&& name) { m_name = std::move(name); }
    void CanSignalDescription::set_name(std::string_view name) { m_name = name; }

    [[nodiscard]] auto CanSignalDescription::bit_start() const -> uint16_t { return m_bit_start; }
    void CanSignalDescription::set_bit_start(uint16_t bit) { m_bit_start = bit; }

    [[nodiscard]] auto CanSignalDescription::bit_length() const -> uint16_t { return m_bit_length; }
    void CanSignalDescription::set_bit_length(uint16_t length) { m_bit_length = length; }

    [[nodiscard]] auto CanSignalDescription::endianness() const -> Endianness { return m_endianness; }
    void CanSignalDescription::set_endianness(Endianness endianness) { m_endianness = endianness; }

    [[nodiscard]] auto CanSignalDescription::data_format() const -> DataFormat { return m_data_format; }
    void CanSignalDescription::set_data_format(DataFormat format) { m_data_format = format; }

    [[nodiscard]] auto CanSignalDescription::factor() const -> double { return m_factor; }
    void CanSignalDescription::set_factor(double factor) { m_factor = factor; }

    [[nodiscard]] auto CanSignalDescription::offset() const -> double { return m_offset; }
    void CanSignalDescription::set_offset(double offset) { m_offset = offset; }

    [[nodiscard]] auto CanSignalDescription::factor_offset_used() const -> bool {
        return m_factor != 1.0 || m_offset != 0.0;
    }
    void CanSignalDescription::clear_factor_offset() {
        m_factor = 1.0;
        m_offset = 0.0;
    }

    [[nodiscard]] auto CanSignalDescription::minimum() const -> double { return m_minimum; }
    void CanSignalDescription::set_minimum(double minimum) { m_minimum = minimum; }

    [[nodiscard]] auto CanSignalDescription::maximum() const -> double { return m_maximum; }
    void CanSignalDescription::set_maximum(double maximum) { m_maximum = maximum; }

    [[nodiscard]] auto CanSignalDescription::minimum_maximum_used() const -> bool {
        return m_minimum != 0.0 || m_maximum != 0.0;
    }
    void CanSignalDescription::clear_minimum_maximum() {
        m_minimum = 0.0;
        m_maximum = 0.0;
    }

    [[nodiscard]] auto CanSignalDescription::unit() const -> std::string { return m_unit; }
    void CanSignalDescription::set_unit(std::string&& unit) { m_unit = std::move(unit); }
    void CanSignalDescription::set_unit(std::string_view unit) { m_unit = unit; }

    [[nodiscard]] auto CanSignalDescription::receiver() const -> std::string { return m_receiver; }
    void CanSignalDescription::set_receiver(std::string&& receiver) { m_receiver = std::move(receiver); }
    void CanSignalDescription::set_receiver(std::string_view receiver) { m_receiver = receiver; }

    [[nodiscard]] auto CanSignalDescription::multiplex_state() const -> MultiplexState { return m_multiplex_state; }
    void CanSignalDescription::set_multiplex_state(MultiplexState state) { m_multiplex_state = state; }

    [[nodiscard]] auto CanSignalDescription::comment() const -> std::string { return m_comment; }
    void CanSignalDescription::set_comment(std::string&& comment) { m_comment = std::move(comment); }
    void CanSignalDescription::set_comment(std::string_view comment) { m_comment = comment; }

    [[nodiscard]] auto CanSignalDescription::is_valid() const -> bool {
        if (m_name.empty() ||
            !(m_bit_length > 0 && m_bit_length <= 512) ||
            (m_data_format == DataFormat::SignedInteger && m_bit_length > 64) ||
            (m_data_format == DataFormat::UnsignedInteger && m_bit_length > 64) ||
            (m_data_format == DataFormat::Float && m_bit_length != 32) ||
            (m_data_format == DataFormat::Double && m_bit_length != 64) ||
            (m_data_format == DataFormat::AsciiString && (m_bit_length % 8 != 0))) {

            return false;
        }
        return true;
    }

    auto operator<<(std::ostream& os, CanSignalDescription const& signal) -> std::ostream& {
        os << "Signal Name: \"" << signal.m_name << "\"\n";
        os << "  Bit Start: " << signal.m_bit_start << "\n";
        os << "  Bit Length: " << signal.m_bit_length << "\n";
        os << "  Endianness: " << (signal.m_endianness == Endianness::BigEndian ? "Big Endian" : "Little Endian") << "\n";
        os << "  Data Format: ";
        switch (signal.m_data_format) {
            case DataFormat::SignedInteger:
                os << "Signed Integer";
                break;
            case DataFormat::UnsignedInteger:
                os << "Unsigned Integer";
                break;
            case DataFormat::Float:
                os << "Float";
                break;
            case DataFormat::Double:
                os << "Double";
                break;
            case DataFormat::AsciiString:
                os << "ASCII String";
                break;
        }
        os << "\n";
        os << "  Factor: " << signal.m_factor << "\n";
        os << "  Offset: " << signal.m_offset << "\n";
        os << "  Minimum: " << signal.m_minimum << "\n";
        os << "  Maximum: " << signal.m_maximum << "\n";
        os << "  Unit: \"" << signal.m_unit << "\"\n";
        os << "  Receiver: \"" << signal.m_receiver << "\"\n";
        os << "  Multiplex State: ";
        switch (signal.m_multiplex_state) {
            case MultiplexState::None:
                os << "None";
                break;
            case MultiplexState::MultiplexorSwitch:
                os << "Multiplexor Switch";
                break;
            case MultiplexState::MultiplexedSignal:
                os << "Multiplexed Signal";
                break;
            case MultiplexState::SwitchAndSignal:
                os << "Switch and Signal";
                break;
        }
        os << "\n";
        os << "  Comment: \"" << signal.m_comment << "\"";
        return os;
    }

} // namespace mrover::dbc_runtime
