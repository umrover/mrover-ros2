#include "frame_processor.hpp"

#include <bit>
#include <concepts>
#include <vector>

namespace mrover::dbc_runtime {
    void CanFrameProcessor::add_message_description(CanMessageDescription message) {
        uint32_t const id = message.id();
        m_message_descriptions.emplace(id, std::move(message));
    }

    auto CanFrameProcessor::decode(uint32_t message_id, std::string_view data) -> std::unordered_map<std::string, CanSignalValue> {
        std::unordered_map<std::string, CanSignalValue> signal_values{};

        auto it = m_message_descriptions.find(message_id);
        if (it == m_message_descriptions.end()) {
            return signal_values;
        }

        CanMessageDescription const& message_desc = it->second;

        for (auto const& signal: message_desc.signals()) {
            auto bit_start = signal.bit_start();
            auto bit_length = signal.bit_length();
            auto signal_value = from_signal_description(data, signal);
            if (signal_value.has_value()) {
                signal_values.emplace(signal.name(), signal_value.value());
            }
        }

        return signal_values;
    }

    auto CanFrameProcessor::encode(std::string const& message_name, std::unordered_map<std::string, CanSignalValue> const& signal_values) -> std::expected<CanFrame, Error> {
        for (auto const& [id, message_desc]: m_message_descriptions) {
            if (message_desc.name() == message_name) {
                std::bitset<512> raw_bitset{};

                for (auto const& [signal_name, signal_value]: signal_values) {
                    CanSignalDescription const* signal_desc = message_desc.signal(signal_name);
                    if (signal_desc == nullptr || !signal_desc->is_valid()) {
                        return std::unexpected(Error::InvalidSignalDescription);
                    }

                    uint64_t raw_value = 0;

                    switch (signal_desc->data_format()) {
                        case DataFormat::SignedInteger: {
                            if (!signal_value.is_numeric()) {
                                return std::unexpected(Error::InvalidSignalValue);
                            }
                            int64_t int_value = signal_value.as_signed_integer();
                            if (signal_desc->factor_offset_used()) {
                                double const raw = (static_cast<double>(int_value) - signal_desc->offset()) / signal_desc->factor();
                                int_value = static_cast<int64_t>(raw);
                            }
                            raw_value = static_cast<uint64_t>(int_value);
                            break;
                        }
                        case DataFormat::UnsignedInteger: {
                            if (!signal_value.is_numeric()) {
                                return std::unexpected(Error::InvalidSignalValue);
                            }
                            uint64_t uint_value = signal_value.as_unsigned_integer();
                            if (signal_desc->factor_offset_used()) {
                                double const raw = (static_cast<double>(uint_value) - signal_desc->offset()) / signal_desc->factor();
                                uint_value = static_cast<uint64_t>(raw);
                            }
                            raw_value = uint_value;
                            break;
                        }
                        case DataFormat::Float: {
                            if (!signal_value.is_numeric()) {
                                return std::unexpected(Error::InvalidSignalValue);
                            }
                            double double_value = signal_value.as_double();
                            if (signal_desc->factor_offset_used()) {
                                double_value = (double_value - signal_desc->offset()) / signal_desc->factor();
                            }
                            auto const float_value = static_cast<float>(double_value);
                            raw_value = static_cast<uint64_t>(std::bit_cast<uint32_t>(float_value));
                            break;
                        }
                        case DataFormat::Double: {
                            if (!signal_value.is_numeric()) {
                                return std::unexpected(Error::InvalidSignalValue);
                            }
                            double double_value = signal_value.as_double();
                            if (signal_desc->factor_offset_used()) {
                                double const raw = (double_value - signal_desc->offset()) / signal_desc->factor();
                                double_value = raw;
                            }
                            raw_value = std::bit_cast<uint64_t>(double_value);
                            break;
                        }
                        case DataFormat::AsciiString:
                            break;
                        default:
                            return std::unexpected(Error::InvalidSignalDescription);
                    }
                    for (size_t i = 0; i < signal_desc->bit_length(); ++i) {
                        if (raw_value & (uint64_t(1) << i)) {
                            raw_bitset.set(signal_desc->bit_start() + i);
                        }
                    }
                } // for (auto const& [signal_name, signal_value]: signal_values)

                std::vector<uint8_t> frame_data(message_desc.length(), 0);
                for (size_t i = 0; i < message_desc.length() * 8; ++i) {
                    if (raw_bitset.test(i)) {
                        frame_data[i / 8] |= (1u << (i % 8));
                    }
                }
                return std::expected<CanFrame, Error>(std::in_place, CanFrame{id, std::move(frame_data)});
            }
        }

        return std::unexpected(Error::InvalidMessageDescription);
    }


    auto CanFrameProcessor::extract_raw_bytes(std::string_view data, CanSignalDescription const& signal) -> std::expected<std::vector<uint8_t>, Error> {
        if (!signal.is_valid()) {
            return std::unexpected(Error::InvalidSignalDescription);
        }

        if (data.size() > 64) {
            return std::unexpected(Error::InvalidDataFrame);
        }

        auto const bit_start = signal.bit_start();
        auto const bit_length = signal.bit_length();

        if (bit_start + bit_length > data.size() * 8) {
            return std::unexpected(Error::InvalidDataFrame);
        }

        std::bitset<512> raw_bitset{};

        auto const* data_ptr = reinterpret_cast<unsigned char const*>(data.data());
        for (size_t byte = 0; byte < data.size(); ++byte) {
            std::uint8_t b = data_ptr[byte];
            for (size_t bit = 0; bit < 8; ++bit) {
                if (b & (1u << bit)) {
                    raw_bitset.set(byte * 8 + bit);
                }
            }
        }

        raw_bitset >>= bit_start;

        std::vector<uint8_t> raw_bytes((bit_length + 7) / 8, 0);
        for (size_t i = 0; i < bit_length; ++i) {
            if (raw_bitset.test(i)) {
                raw_bytes[i / 8] |= (1u << (i % 8));
            }
        }

        return std::expected<std::vector<uint8_t>, Error>(std::in_place, std::move(raw_bytes));
    }

    // ===== HELPERS FOR from_signal_description =====
    constexpr auto bytes_to_int(std::span<uint8_t const> bytes) -> uint64_t {
        if (bytes.size() > 8) {
            return 0;
        }

        uint64_t value = 0;
        for (std::size_t i = 0; i < bytes.size(); ++i) {
            value |= static_cast<uint64_t>(bytes[i]) << (8 * i); // little-endian
        }
        return value;
    }

    constexpr auto do_factor_and_offset(double raw, double factor, double offset) -> double {
        return raw * factor + offset;
    }

    template<typename T>
    concept numeric = std::integral<T> || std::floating_point<T>;

    constexpr auto get_physical_value(numeric auto raw, CanSignalDescription const& signal) -> CanSignalValue {
        if (signal.factor_offset_used()) {
            double physical_value = do_factor_and_offset(raw, signal.factor(), signal.offset());
            return physical_value;
        }

        return raw;
    }
    // ===== END HELPERS =============================

    auto CanFrameProcessor::from_signal_description(std::string_view data, CanSignalDescription const& signal) -> std::expected<CanSignalValue, Error> {
        auto bytes_result = extract_raw_bytes(data, signal);
        if (!bytes_result.has_value()) {
            return std::unexpected(bytes_result.error());
        }

        auto const& bytes = bytes_result.value();
        auto const bit_length = signal.bit_length();

        if (bit_length == 0 || bit_length > 64) {
            return std::unexpected(Error::InvalidSignalDescription);
        }

        switch (signal.data_format()) {
            case DataFormat::SignedInteger: {
                if (bytes.size() > 8) {
                    return std::unexpected(Error::InvalidDataFrame);
                }

                uint64_t raw_u = bytes_to_int(bytes);

                if (bit_length < 64) {
                    raw_u &= (uint64_t(1) << bit_length) - 1;
                }

                int64_t raw_s;
                if (bit_length == 64) {
                    raw_s = static_cast<int64_t>(raw_u);
                } else {
                    uint64_t sign_bit = uint64_t(1) << (bit_length - 1);
                    if (raw_u & sign_bit) {
                        raw_u |= ~((uint64_t(1) << bit_length) - 1);
                    }
                    raw_s = static_cast<int64_t>(raw_u);
                }

                return std::expected<CanSignalValue, Error>(std::in_place, get_physical_value(raw_s, signal));
            }

            case DataFormat::UnsignedInteger: {
                if (bytes.size() > 8) {
                    return std::unexpected(Error::InvalidDataFrame);
                }

                uint64_t raw_u = bytes_to_int(bytes);

                if (bit_length < 64) {
                    raw_u &= (uint64_t(1) << bit_length) - 1;
                }

                return std::expected<CanSignalValue, Error>(std::in_place, get_physical_value(raw_u, signal));
            }

            case DataFormat::Float: {
                if (bytes.size() != 4) {
                    return std::unexpected(Error::InvalidDataFrame);
                }

                auto raw_int = static_cast<uint32_t>(bytes_to_int(bytes));
                float raw_value;
                std::memcpy(&raw_value, &raw_int, sizeof(float));

                return std::expected<CanSignalValue, Error>(std::in_place, get_physical_value(raw_value, signal));
            }

            case DataFormat::Double: {
                if (bytes.size() != 8) {
                    return std::unexpected(Error::InvalidDataFrame);
                }

                auto raw_int = static_cast<uint64_t>(bytes_to_int(bytes));
                double raw_value;
                std::memcpy(&raw_value, &raw_int, sizeof(double));

                return std::expected<CanSignalValue, Error>(std::in_place, get_physical_value(raw_value, signal));
            }

            case DataFormat::AsciiString: {
                std::string str_value(bytes.begin(), bytes.end());
                return std::expected<CanSignalValue, Error>(
                        std::in_place, CanSignalValue{std::move(str_value)});
            }
        }

        return std::unexpected(Error::InvalidSignalDescription);
    }

} // namespace mrover::dbc_runtime
