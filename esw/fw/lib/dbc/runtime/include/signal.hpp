#pragma once

#include <cstdint>
#include <ostream>
#include <string>
#include <variant>

namespace mrover::dbc_runtime {

    enum class Endianness {
        BigEndian = 0,
        LittleEndian = 1,
    };

    enum class DataFormat {
        SignedInteger = 0,
        UnsignedInteger = 1,
        Float = 2,
        Double = 3,
        AsciiString = 4,
    };

    enum class MultiplexState {
        None = 0x00,
        MultiplexorSwitch = 0x01,
        MultiplexedSignal = 0x02,
        SwitchAndSignal = MultiplexorSwitch | MultiplexedSignal, // if extended multiplexing is supported
    };

    class CanSignalValue : public std::variant<int8_t, uint8_t, int16_t, uint16_t,
                                               int32_t, uint32_t, int64_t, uint64_t,
                                               float, double, std::string> {
    public:
        using variant::variant;

        [[nodiscard]] auto is_integral() const -> bool;
        [[nodiscard]] auto is_floating_point() const -> bool;
        [[nodiscard]] auto is_numeric() const -> bool;
        [[nodiscard]] auto is_string() const -> bool;

        [[nodiscard]] auto as_signed_integer() const -> int64_t;
        [[nodiscard]] auto as_unsigned_integer() const -> uint64_t;
        [[nodiscard]] auto as_double() const -> double;
        [[nodiscard]] auto as_string() const -> std::string;

        friend auto operator<<(std::ostream& os, CanSignalValue const& value) -> std::ostream&;
    };

    class CanSignalDescription {
    public:
        friend class CanMessageDescription;

        CanSignalDescription() = default;

        [[nodiscard]] auto name() const -> std::string;
        void set_name(std::string&& name);
        void set_name(std::string_view name);

        [[nodiscard]] auto bit_start() const -> uint16_t;
        void set_bit_start(uint16_t bit);

        [[nodiscard]] auto bit_length() const -> uint16_t;
        void set_bit_length(uint16_t length);

        [[nodiscard]] auto endianness() const -> Endianness;
        void set_endianness(Endianness endianness);

        [[nodiscard]] auto data_format() const -> DataFormat;
        void set_data_format(DataFormat format);

        [[nodiscard]] auto factor() const -> double;
        void set_factor(double factor);

        [[nodiscard]] auto offset() const -> double;
        void set_offset(double offset);

        [[nodiscard]] auto factor_offset_used() const -> bool;
        void clear_factor_offset();

        [[nodiscard]] auto minimum() const -> double;
        void set_minimum(double minimum);

        [[nodiscard]] auto maximum() const -> double;
        void set_maximum(double maximum);

        [[nodiscard]] auto minimum_maximum_used() const -> bool;
        void clear_minimum_maximum();

        [[nodiscard]] auto unit() const -> std::string;
        void set_unit(std::string&& unit);
        void set_unit(std::string_view unit);

        [[nodiscard]] auto receiver() const -> std::string;
        void set_receiver(std::string&& receiver);
        void set_receiver(std::string_view receiver);

        [[nodiscard]] auto multiplex_state() const -> MultiplexState;
        void set_multiplex_state(MultiplexState state);

        [[nodiscard]] auto comment() const -> std::string;
        void set_comment(std::string&& comment);
        void set_comment(std::string_view comment);

        [[nodiscard]] auto is_valid() const -> bool;

        friend auto operator<<(std::ostream& os, CanSignalDescription const& signal) -> std::ostream&;

    private:
        std::string m_name{};
        uint16_t m_bit_start{};
        uint16_t m_bit_length{};
        Endianness m_endianness{};
        DataFormat m_data_format{};
        double m_factor = 1.0;
        double m_offset = 0.0;
        double m_minimum = 0.0;
        double m_maximum = 0.0;
        std::string m_unit{};
        std::string m_receiver{};
        MultiplexState m_multiplex_state = MultiplexState::None;
        std::string m_comment{};
    };

} // namespace mrover::dbc_runtime
