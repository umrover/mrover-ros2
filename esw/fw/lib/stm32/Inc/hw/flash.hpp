#pragma once

#include <cstdint>

namespace mrover {

    template<typename T>
    concept mem_layout_t = requires {
        { T::FLASH_BEGIN_ADDR } -> std::convertible_to<uint32_t>;
        { T::FLASH_END_ADDR } -> std::convertible_to<uint32_t>;
        { T::PAGE_SIZE } -> std::convertible_to<int>;
        { T::NUM_PAGES } -> std::convertible_to<int>;
    };

    template<typename T>
    concept config_t = requires {
        { std::declval<T>().all() };
        typename std::tuple_size<std::remove_cvref_t<decltype(std::declval<T>().all())>>::type;
    };

    template<config_t Config>
    struct config_validator_t {
    private:
        static constexpr size_t N = std::tuple_size_v<decltype(std::declval<Config>().all())>;

        struct reg_descriptor_t {
            uint16_t addr{};
            size_t size{};
            [[nodiscard]] constexpr auto reg() const -> uint16_t { return addr; }
        };

        template<std::size_t... Is>
        static constexpr auto tuple_to_array(std::index_sequence<Is...>) {
            auto cfg = Config{};
            auto tup = cfg.all();
            return std::array<reg_descriptor_t, N>{
                    reg_descriptor_t{
                            std::get<Is>(tup).addr,
                            std::get<Is>(tup).size()}...};
        }

        static constexpr auto fields = [] -> auto {
            auto tmp = tuple_to_array(std::make_index_sequence<N>{});
            std::sort(tmp.begin(), tmp.end(),
                      [](auto const& a, auto const& b) -> auto { return a.reg() < b.reg(); });
            return tmp;
        }();

    public:
        static consteval auto is_valid() -> bool {
            if constexpr (N == 0) return false;
            for (size_t i = 1; i < N; ++i) {
                auto const& prev = fields[i - 1];
                auto const& curr = fields[i];
                if (curr.addr < prev.addr + prev.size) return false;
            }
            return true;
        }

        static consteval auto size_bytes() -> uint16_t {
            if constexpr (N == 0) return 0;
            auto constexpr last_field = fields[N - 1];
            return last_field.addr + last_field.size;
        }
    };

    template<typename Config>
    struct validated_config_t {
        static consteval auto is_valid() -> bool {
            return config_validator_t<Config>::is_valid();
        }

        static consteval auto size_bytes() -> uint16_t {
            return config_validator_t<Config>::size_bytes();
        }
    };

    template<typename T>
    static auto from_raw(uint32_t raw) -> T {
        static_assert(std::is_trivially_copyable_v<T>);
        if constexpr (sizeof(T) == sizeof(uint32_t)) {
            return std::bit_cast<T>(raw);
        } else {
            return static_cast<T>(raw);
        }
    }

    template<typename T>
    static auto to_raw(T value) -> uint32_t {
        static_assert(std::is_trivially_copyable_v<T>);
        if constexpr (sizeof(T) == sizeof(uint32_t)) {
            return std::bit_cast<uint32_t>(value);
        } else {
            return static_cast<uint32_t>(value);
        }
    }

    template<typename T>
    struct reg_t {
        using value_t = T;
        uint8_t addr{};
        mutable std::optional<T> value = std::nullopt;
        static consteval auto size() -> size_t { return sizeof(T); }
        [[nodiscard]] constexpr auto reg() const -> uint8_t { return addr; }

        template<typename Config>
        static constexpr auto get_base_addr() -> uint32_t {
            using Mem = Config::mem_layout;
            return Mem::FLASH_BEGIN_ADDR + (Mem::PAGE_SIZE * (Mem::NUM_PAGES - 1));
        }

        template<typename Config>
        auto read(Config const& cfg) const -> T {
            if (!value.has_value()) {
                uint32_t const phys_addr = get_base_addr<Config>() + addr;
                T temp;
                std::memcpy(&temp, reinterpret_cast<void*>(phys_addr), sizeof(T));
                value = temp;
            }
            return *value;
        }

        template<typename Config>
        void write(Config const& cfg, T v) const {
            using Mem = Config::mem_layout;
            value = v;

            uint32_t const page_addr = get_base_addr<Config>();
            static uint8_t page_buffer[Mem::PAGE_SIZE];

            std::memcpy(page_buffer, reinterpret_cast<void*>(page_addr), Mem::PAGE_SIZE);
            std::memcpy(&page_buffer[addr], &v, sizeof(T));

            HAL_FLASH_Unlock();
            FLASH_EraseInitTypeDef erase_init;
            erase_init.TypeErase = FLASH_TYPEERASE_PAGES;
            erase_init.Page = (page_addr - 0x08000000) / Mem::PAGE_SIZE;
            erase_init.NbPages = 1;

            uint32_t page_error = 0;
            if (HAL_FLASHEx_Erase(&erase_init, &page_error) == HAL_OK) {
                for (size_t i = 0; i < Mem::PAGE_SIZE; i += 8) {
                    uint64_t double_word;
                    std::memcpy(&double_word, &page_buffer[i], 8);
                    if (double_word != 0xFFFFFFFFFFFFFFFF) {
                        HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, page_addr + i, double_word);
                    }
                }
            }
            HAL_FLASH_Lock();
        }
    };

    template<auto cfg_ptr_v, size_t bit = 0, size_t width = 0>
    struct field_t {
        static constexpr auto reg_ptr = cfg_ptr_v;

        template<typename C>
        using reg_type = std::remove_reference_t<decltype(std::declval<C>().*cfg_ptr_v)>;

        template<typename C>
        using T = reg_type<C>::value_t;

        template<typename Config>
        static auto get(Config const& cfg) {
            using V = T<Config>;
            auto const& reg = cfg.*cfg_ptr_v;
            V raw = reg.read(cfg);

            if constexpr (std::is_floating_point_v<V>) {
                return raw;
            } else {
                constexpr size_t w = (width == 0) ? 1 : width;

                if constexpr (w == 1) {
                    return static_cast<bool>((raw >> bit) & 1);
                } else {
                    constexpr V mask = (static_cast<V>(1) << w) - 1;
                    return static_cast<V>((raw >> bit) & mask);
                }
            }
        }

        template<typename Config, typename Value>
        static void set(Config const& cfg, Value v) {
            using V = T<Config>;
            auto const& reg = cfg.*cfg_ptr_v;

            if constexpr (std::is_floating_point_v<V>) {
                reg.write(cfg, static_cast<V>(v));
            } else {
                constexpr size_t w = (width == 0) ? 1 : width;
                V current = reg.read(cfg);
                constexpr V mask = ((static_cast<V>(1) << w) - 1) << bit;
                V updated = (current & ~mask) | ((static_cast<V>(v) << bit) & mask);
                reg.write(cfg, updated);
            }
        }
    };

} // namespace mrover
