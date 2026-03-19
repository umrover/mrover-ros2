#include "hw/flash.hpp"
#include "main.h"
#include "stm32g4xx_hal_gpio.h"

extern "C" {
#include <stdio.h>
}

namespace mrover {

    struct bmc_config_t {
        static constexpr reg_t<uint8_t> CAN_ID{"can_id", 0x00};
        static constexpr reg_t<uint8_t> LIMITS_ENABLED{"limits_enabled", 0x01};
        static constexpr reg_t<uint16_t> INT_VALUE{"int_value", 0x02};
        static constexpr reg_t<float> FLOAT_VALUE{"float_value", 0x08};

        static constexpr auto all() {
            return std::make_tuple(CAN_ID, LIMITS_ENABLED, INT_VALUE, FLOAT_VALUE);
        }

        static consteval uint16_t size_bytes() {
            return validated_config_t<bmc_config_t>::size_bytes();
        }
    };

    struct G431RB {

        static constexpr uint32_t FLASH_BEGIN_ADDR = 0x08000000;
        static constexpr uint32_t FLASH_END_ADDR = 0x0801FFFF;
        static constexpr uint32_t LAST_PAGE_START = 0x0801F800;
        static constexpr int PAGE_SIZE = 2048;
        static constexpr int NUM_PAGES = 64;
    };

    Flash<bmc_config_t, G431RB> flash(G431RB::LAST_PAGE_START);

    int count = 0;
    uint32_t start_addr;
    uint32_t curr_addr;
    uint64_t read = 0;

    uint32_t read_32;
    uint16_t read_16;
    uint8_t read_8;

    auto init() -> void {
        printf("\n\r===== RESET =====\n\r");
        start_addr = (uint32_t) flash.m_region_start;
        // printf("Init start_addr: %lx\n\r", start_addr);
        curr_addr = start_addr;
        // printf("Init curr_addr: %lx\n\r", curr_addr);

        //flash.write_config(bmc_config_t::CAN_ID, 0x02);
        auto const id = flash.read_config(bmc_config_t::CAN_ID);
        printf("CAN ID: 0x%u \n\r", id);
        //flash.write_config(bmc_config_t::LIMITS_ENABLED, 0x20);
        auto const lims_enabled = flash.read_config(bmc_config_t::LIMITS_ENABLED);
        printf("LIMITS ENABLED: 0x%u \n\r", lims_enabled);
        //flash.write_config(bmc_config_t::INT_VALUE, 12);
        auto const int_value = flash.read_config(bmc_config_t::INT_VALUE);
        printf("INT VALUE : %u \n\r", int_value);
        //flash.write_config(bmc_config_t::FLOAT_VALUE, 3.1);
        auto const float_value = flash.read_config(bmc_config_t::FLOAT_VALUE);
        printf("FLOAT VALUE : %.5f \n\r", float_value);
    }

    auto loop() -> void {

        printf("====== LOOP ======\n\r");

        while (true) {

            int button_state = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);

            if (button_state && count < 256) {

                printf("COUNT: %u\n\r", count);

                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
                HAL_Delay(100);
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

                if (count == 0) {
                    flash.write_config(bmc_config_t::CAN_ID, 0x03);
                    uint8_t can = flash.read_config(bmc_config_t::CAN_ID);
                    printf("CAN ID IS NOW: 0x%X\n\r", can);
                }

                if (count == 1) {
                    flash.write_config(bmc_config_t::FLOAT_VALUE, 2.718);
                    auto eulers = flash.read_config(bmc_config_t::FLOAT_VALUE);
                    uint32_t float_as_32;
                    memcpy(&float_as_32, &eulers, sizeof(float));
                    printf("FLOAT VALUE IS NOW: %.5f\n\r", eulers);
                }

                ++count;

            } // if button state

        } // while true

    } // loop

} // namespace mrover

extern "C" {
void PostInit() {
    mrover::init();
}

void Loop() {
    mrover::loop();
}
}
