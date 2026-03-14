#pragma once

#include <serial/fdcan.hpp>
#include <serial/uart.hpp>
#include <CANBus1.hpp>
#include <adc.hpp>
#include <hw/flash.hpp>


namespace mrover {
    // science board config
    struct sb_config_t {
        reg_t<uint8_t> CAN_ID{0x00};
        reg_t<uint8_t> HOST_CAN_ID{0x01};


        using can_id = field_t<&sb_config_t::CAN_ID, 0, 8>; // retrieves CAN_ID from bits 0-7 of CAN_ID register
        using host_can_id = field_t<&sb_config_t::HOST_CAN_ID, 0, 8>; // retrieves HOST_CAN_ID from bits 0-7 of HOST_CAN_ID register

        template<typename F>
        auto get() const { return F::get(*this); }

        // stm32 g431cbt6
        struct mem_layout {
            static constexpr uint32_t FLASH_BEGIN_ADDR = 0x08000000;
            static constexpr uint32_t FLASH_END_ADDR = 0x0801FFFF;
            static constexpr int PAGE_SIZE = 2048;
            static constexpr int NUM_PAGES = 64;
        };
    };

    // retrieves science board uart options
    inline auto get_uart_options() -> UART::Options {
        UART::Options options;
        options.use_dma = false;
        return options;
    }

    // retrieves science board can options
    inline auto get_can_options() -> FDCAN::Options {
        auto can_opts = FDCAN::Options{};
        can_opts.delay_compensation = true;
        can_opts.tdc_offset = 13;
        can_opts.tdc_filter = 1;
        return can_opts;
    }

    // retrieves science board adc options
    inline auto get_adc_options() -> ADCBase::Options {
        ADCBase::Options options;
        options.use_dma = true;
        return options;
    }
} // namespace mrover
