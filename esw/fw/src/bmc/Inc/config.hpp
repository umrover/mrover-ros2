#pragma once

#include <serial/fdcan.hpp>
#include <serial/uart.hpp>
#include <tuple>

#include <MRoverCAN.hpp>
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
    };

    struct bmc_config_t {
        static inline void* flash_ptr = nullptr;

        FDCAN::Filter can_node_filter{};

        reg_t<uint8_t> CAN_ID{0x00};
        reg_t<uint8_t> HOST_CAN_ID{0x01};
        reg_t<uint8_t> SYS_CFG{0x02};
        reg_t<uint8_t> LIMIT_CFG{0x03};
        reg_t<float> QUAD_CPR{0x04};
        reg_t<float> GEAR_RATIO{0x08};
        reg_t<float> ROTOR_OUTPUT_RATIO{0x0C};
        reg_t<float> LIMIT_A_POSITION{0x10};
        reg_t<float> LIMIT_B_POSITION{0x14};
        reg_t<float> MAX_PWM{0x18};
        reg_t<float> MIN_POS{0x1C};
        reg_t<float> MAX_POS{0x20};
        reg_t<float> MIN_VEL{0x24};
        reg_t<float> MAX_VEL{0x28};
        reg_t<float> POS_K_P{0x2C};
        reg_t<float> POS_K_I{0x30};
        reg_t<float> POS_K_D{0x34};
        reg_t<float> POS_K_F{0x38};
        reg_t<float> VEL_K_P{0x3C};
        reg_t<float> VEL_K_I{0x40};
        reg_t<float> VEL_K_D{0x44};
        reg_t<float> VEL_K_F{0x48};
        reg_t<float> STALL_CURRENT{0x4C};
        reg_t<float> DELTA_POSITION{0x50};

        using can_id = field_t<&bmc_config_t::CAN_ID, 0, 8>;
        using host_can_id = field_t<&bmc_config_t::HOST_CAN_ID, 0, 8>;

        using motor_en = field_t<&bmc_config_t::SYS_CFG, 0>;
        using motor_inv = field_t<&bmc_config_t::SYS_CFG, 1>;
        using quad_en = field_t<&bmc_config_t::SYS_CFG, 2>;
        using quad_phase = field_t<&bmc_config_t::SYS_CFG, 3>;
        using stall_en = field_t<&bmc_config_t::SYS_CFG, 4>;

        using lim_a_en = field_t<&bmc_config_t::LIMIT_CFG, 0>;
        using lim_a_active_high = field_t<&bmc_config_t::LIMIT_CFG, 1>;
        using lim_a_is_forward = field_t<&bmc_config_t::LIMIT_CFG, 2>;
        using lim_a_use_readjust = field_t<&bmc_config_t::LIMIT_CFG, 3>;
        using lim_b_en = field_t<&bmc_config_t::LIMIT_CFG, 4>;
        using lim_b_active_high = field_t<&bmc_config_t::LIMIT_CFG, 5>;
        using lim_b_is_forward = field_t<&bmc_config_t::LIMIT_CFG, 6>;
        using lim_b_use_readjust = field_t<&bmc_config_t::LIMIT_CFG, 7>;

        using quad_cpr = field_t<&bmc_config_t::QUAD_CPR>;
        using gear_ratio = field_t<&bmc_config_t::GEAR_RATIO>;
        using rotor_output_ratio = field_t<&bmc_config_t::ROTOR_OUTPUT_RATIO>;
        using limit_a_position = field_t<&bmc_config_t::LIMIT_A_POSITION>;
        using limit_b_position = field_t<&bmc_config_t::LIMIT_B_POSITION>;
        using max_pwm = field_t<&bmc_config_t::MAX_PWM>;
        using min_pos = field_t<&bmc_config_t::MIN_POS>;
        using max_pos = field_t<&bmc_config_t::MAX_POS>;
        using min_vel = field_t<&bmc_config_t::MIN_VEL>;
        using max_vel = field_t<&bmc_config_t::MAX_VEL>;
        using pos_k_p = field_t<&bmc_config_t::POS_K_P>;
        using pos_k_i = field_t<&bmc_config_t::POS_K_I>;
        using pos_k_d = field_t<&bmc_config_t::POS_K_D>;
        using pos_k_f = field_t<&bmc_config_t::POS_K_F>;
        using vel_k_p = field_t<&bmc_config_t::VEL_K_P>;
        using vel_k_i = field_t<&bmc_config_t::VEL_K_I>;
        using vel_k_d = field_t<&bmc_config_t::VEL_K_D>;
        using vel_k_f = field_t<&bmc_config_t::VEL_K_F>;
        using stall_current = field_t<&bmc_config_t::STALL_CURRENT>;
        using delta_position = field_t<&bmc_config_t::DELTA_POSITION>;


        template<typename F>
        auto get() const { return F::get(*this); }

        template<typename F>
        void set(auto value) { F::set(*this, value); }

        constexpr auto all() {
            return std::forward_as_tuple(
                    CAN_ID, HOST_CAN_ID, SYS_CFG, LIMIT_CFG, QUAD_CPR, GEAR_RATIO, ROTOR_OUTPUT_RATIO,
                    LIMIT_A_POSITION, LIMIT_B_POSITION, MAX_PWM, MIN_POS, MAX_POS, MIN_VEL, MAX_VEL,
                    POS_K_P, POS_K_I, POS_K_D, POS_K_F, VEL_K_P, VEL_K_I, VEL_K_D, VEL_K_F,
                    STALL_CURRENT, DELTA_POSITION);
        }

        constexpr auto all() const {
            return std::forward_as_tuple(
                    CAN_ID, HOST_CAN_ID, SYS_CFG, LIMIT_CFG, QUAD_CPR, GEAR_RATIO, ROTOR_OUTPUT_RATIO,
                    LIMIT_A_POSITION, LIMIT_B_POSITION, MAX_PWM, MIN_POS, MAX_POS, MIN_VEL, MAX_VEL,
                    POS_K_P, POS_K_I, POS_K_D, POS_K_F, VEL_K_P, VEL_K_I, VEL_K_D, VEL_K_F,
                    STALL_CURRENT, DELTA_POSITION);
        }

        auto set_raw(uint8_t address, uint32_t const raw) -> bool {
            bool found = false;

            std::apply(
                    [&](auto const&... reg) -> void {
                        (
                                [&] -> void {
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
            std::apply([&](auto const&... reg) -> void {
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

        static consteval auto size_bytes() -> uint16_t {
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
        options.use_dma = true;
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
        options.shunt_resistance = 0.0125f;
        options.vref = 3.3f;
        options.vcm = 1.598f;
        options.adc_resolution = 4095;
        return options;
    }

} // namespace mrover
