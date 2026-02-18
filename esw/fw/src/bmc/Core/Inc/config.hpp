#pragma once

#include <serial/fdcan.hpp>
#include <serial/uart.hpp>
#include <tuple>

#include <CANBus1.hpp>
#include <adc.hpp>
#include <hw/ad8418a.hpp>
#include <hw/flash.hpp>

namespace mrover {

    enum struct mode_t : uint8_t {
        STOPPED = 0,
        FAULT = 1,
        THROTTLE = 5,
        POSITION = 6,
        VELOCITY = 7,
    };

    enum struct encoder_mode_t : uint8_t {
        NONE = 0,
        QUAD,
        ABS_SPI,
        ABS_I2C,
    };

    struct bmc_config_t {
        static inline void* flash_ptr = nullptr;

        FDCAN::Filter can_node_filter{};

        reg_t<uint8_t> CAN_ID{"can_id", 0x00};
        reg_t<uint8_t> SYS_CFG{"system_configuration", 0x01};
        reg_t<uint8_t> LIMIT_CFG{"limit_configuration", 0x02};
        reg_t<uint8_t> USER_REG{"user_reg", 0x03};
        reg_t<float> QUAD_CPR{"quad_cpr", 0x04};
        reg_t<float> ABS_I2C_RATIO{"abs_i2c_ratio", 0x08};
        reg_t<float> ABC_I2C_OFFSET{"abs_i2c_offset", 0x0C};
        reg_t<float> ABS_SPI_RATIO{"abs_spi_ratio", 0x10};
        reg_t<float> ABS_SPI_OFFSET{"abs_spi_offset", 0x14};
        reg_t<float> GEAR_RATIO{"gear_ratio", 0x18};
        reg_t<float> LIMIT_A_POSITION{"limit_a_readjust_pos", 0x1C};
        reg_t<float> LIMIT_B_POSITION{"limit_b_readjust_pos", 0x20};
        reg_t<float> MAX_PWM{"max_pwm", 0x24};
        reg_t<float> MIN_POS{"min_pos", 0x28};
        reg_t<float> MAX_POS{"max_pos", 0x2C};
        reg_t<float> MIN_VEL{"min_vel", 0x30};
        reg_t<float> MAX_VEL{"max_vel", 0x34};
        reg_t<float> K_P{"kp", 0x38};
        reg_t<float> K_I{"ki", 0x3C};
        reg_t<float> K_D{"kd", 0x40};
        reg_t<float> K_F{"kf", 0x44};
        reg_t<uint8_t> HOST_CAN_ID{"host_can_id", 0x48};
        reg_t<float> SCALAR{"scalar", 0x49};

        using can_id = field_t<&bmc_config_t::CAN_ID, 0, 8>;

        using motor_en = field_t<&bmc_config_t::SYS_CFG, 0>;
        using motor_inv = field_t<&bmc_config_t::SYS_CFG, 1>;
        using quad_en = field_t<&bmc_config_t::SYS_CFG, 2>;
        using quad_phase = field_t<&bmc_config_t::SYS_CFG, 3>;
        using abs_i2c_en = field_t<&bmc_config_t::SYS_CFG, 4>;
        using abs_i2c_phase = field_t<&bmc_config_t::SYS_CFG, 5>;
        using abs_spi_en = field_t<&bmc_config_t::SYS_CFG, 6>;
        using abs_spi_phase = field_t<&bmc_config_t::SYS_CFG, 7>;

        using lim_a_en = field_t<&bmc_config_t::LIMIT_CFG, 0>;
        using lim_a_active_high = field_t<&bmc_config_t::LIMIT_CFG, 1>;
        using lim_a_is_forward = field_t<&bmc_config_t::LIMIT_CFG, 2>;
        using lim_a_use_readjust = field_t<&bmc_config_t::LIMIT_CFG, 3>;
        using lim_b_en = field_t<&bmc_config_t::LIMIT_CFG, 4>;
        using lim_b_active_high = field_t<&bmc_config_t::LIMIT_CFG, 5>;
        using lim_b_is_forward = field_t<&bmc_config_t::LIMIT_CFG, 6>;
        using lim_b_use_readjust = field_t<&bmc_config_t::LIMIT_CFG, 7>;

        using quad_cpr = field_t<&bmc_config_t::QUAD_CPR>;
        using abs_i2c_ratio = field_t<&bmc_config_t::ABS_I2C_RATIO>;
        using abs_i2c_offset = field_t<&bmc_config_t::ABC_I2C_OFFSET>;
        using abs_spi_ratio = field_t<&bmc_config_t::ABS_SPI_RATIO>;
        using abs_spi_offset = field_t<&bmc_config_t::ABS_SPI_OFFSET>;
        using gear_ratio = field_t<&bmc_config_t::GEAR_RATIO>;
        using scalar = field_t<&bmc_config_t::SCALAR>;
        using limit_a_position = field_t<&bmc_config_t::LIMIT_A_POSITION>;
        using limit_b_position = field_t<&bmc_config_t::LIMIT_B_POSITION>;
        using max_pwm = field_t<&bmc_config_t::MAX_PWM>;
        using min_pos = field_t<&bmc_config_t::MIN_POS>;
        using max_pos = field_t<&bmc_config_t::MAX_POS>;
        using min_vel = field_t<&bmc_config_t::MIN_VEL>;
        using max_vel = field_t<&bmc_config_t::MAX_VEL>;
        using k_p = field_t<&bmc_config_t::K_P>;
        using k_i = field_t<&bmc_config_t::K_I>;
        using k_d = field_t<&bmc_config_t::K_D>;
        using k_f = field_t<&bmc_config_t::K_F>;

        using host_can_id = field_t<&bmc_config_t::HOST_CAN_ID, 0, 8>;

        template<typename F>
        auto get() const { return F::get(*this); }

        template<typename F>
        void set(auto value) { F::set(*this, value); }

        constexpr auto all() {
            return std::forward_as_tuple(
                    CAN_ID, SYS_CFG, LIMIT_CFG, USER_REG, QUAD_CPR, ABS_I2C_RATIO,
                    ABC_I2C_OFFSET, ABS_SPI_RATIO, ABS_SPI_OFFSET, GEAR_RATIO, SCALAR,
                    LIMIT_A_POSITION, LIMIT_B_POSITION, MAX_PWM,
                    MIN_POS, MAX_POS, MIN_VEL, MAX_VEL, K_P, K_I, K_D, K_F, HOST_CAN_ID);
        }

        constexpr auto all() const {
            return std::forward_as_tuple(
                    CAN_ID, SYS_CFG, LIMIT_CFG, USER_REG, QUAD_CPR, ABS_I2C_RATIO,
                    ABC_I2C_OFFSET, ABS_SPI_RATIO, ABS_SPI_OFFSET, GEAR_RATIO, SCALAR,
                    LIMIT_A_POSITION, LIMIT_B_POSITION, MAX_PWM,
                    MIN_POS, MAX_POS, MIN_VEL, MAX_VEL, K_P, K_I, K_D, K_F, HOST_CAN_ID);
        }

        auto set_raw(uint8_t address, uint32_t const raw) -> bool {
            bool found = false;

            std::apply(
                    [&](auto const&... reg) {
                        (
                                [&] {
                                    if (reg.addr == address) {
                                        using T = std::remove_reference_t<decltype(reg)>::value_t;
                                        reg.write(*this, from_raw<T>(raw));
                                        found = true;
                                    }
                                }(),
                                ...);
                    },
                    all());

            return found;
        }

        auto get_raw(uint8_t address, uint32_t& raw) const -> bool {
            bool found = false;
            std::apply([&](auto const&... reg) {
                ((reg.addr == address ? (raw = to_raw(reg.value.value_or(0)), found = true) : false), ...);
            },
                       all());
            return found;
        }

        // stm32 g431cbt6
        struct mem_layout {
            static constexpr uint32_t FLASH_BEGIN_ADDR = 0x08000000;
            static constexpr uint32_t FLASH_END_ADDR = 0x0801FFFF;
            static constexpr int PAGE_SIZE = 2048;
            static constexpr int NUM_PAGES = 64;
        };

        static consteval uint16_t size_bytes() {
            return validated_config_t<bmc_config_t>::size_bytes();
        }
    };

    /**
     * Get the BMC CAN settings.
     *
     * Delay compensation needs to be enabled to allow BRS.
     *
     * @return CAN options for BMC
     */
    inline auto get_can_options(bmc_config_t* config) -> FDCAN::Options {
        config->can_node_filter.id1 = config->get<bmc_config_t::can_id>();
        config->can_node_filter.id2 = CAN_DEST_ID_MASK;
        config->can_node_filter.id_type = FDCAN::FilterIdType::Extended;
        config->can_node_filter.action = FDCAN::FilterAction::Accept;
        config->can_node_filter.mode = FDCAN::FilterMode::Mask;

        FDCAN::FilterConfig filter;
        filter.begin = &config->can_node_filter;
        filter.end = &config->can_node_filter + 1;
        filter.global_non_matching_std_action = FDCAN::FilterAction::Reject;
        filter.global_non_matching_ext_action = FDCAN::FilterAction::Reject;

        auto can_opts = FDCAN::Options{};
        can_opts.delay_compensation = true;
        can_opts.tdc_offset = 13;
        can_opts.tdc_filter = 1;
        can_opts.filter_config = filter;
        return can_opts;
    }

    /**
     * Get the BMC UART settings.
     *
     * DMA must be enabled to allow non-blocking logging functionality.
     *
     * @return UART options for BMC
     */
    inline auto get_uart_options() -> UART::Options {
        UART::Options options;
        options.use_dma = false;
        return options;
    }

    /**
     * Get the BMC ADC settings.
     *
     * DMA must be enabled to allow non-blocking ADC functionality.
     *
     * @return ADC options for BMC
     */
    inline auto get_adc_options() -> ADCBase::Options {
        ADCBase::Options options;
        options.use_dma = false; // TODO(eric) use dma for this
        return options;
    }

    inline auto get_current_sensor_options() -> AD8418A::Options {
        AD8418A::Options options;
        options.gain = 20.0f;
        options.shunt_resistance = 0.0005f;
        options.vref = 3.3f;
        options.vcm = options.vref / 2.0f;
        options.adc_resolution = 4095;
        return options;
    }

} // namespace mrover
